/*
 * PETANQUE LIDAR PROTOTYPE
 * -------------------------
 * Simulates two LiDAR units mounted on 3m poles on opposite sides of a
 * petanque terrain, angled down at the field. The scanners do NOT get
 * told where the balls/jack are: they only get raw (x,y,z,intensity,
 * inFieldBounds) returns, exactly like real sensors.
 *
 * Pipeline implemented here (steppable with 1/2/3/4 or LEFT/RIGHT):
 *   1. Ground-truth scene: flat rectangular terrain, N boules (metal spheres)
 *      and 1 cochonnet/jack (small wooden sphere), randomly placed.
 *   2. LidarScan(): raycast over an azimuth/elevation raster from each
 *      scanner pole, intersect against ground plane + all spheres,
 *      keep nearest hit, tag material (affects simulated reflectivity).
 *      Two poles from opposite sides reduce the shadowing that a single
 *      viewpoint would suffer when one ball occludes another.
 *   3. BuildGrid(): rasterize the returns into a top-down height/intensity
 *      grid over the field footprint only (points outside the field
 *      rectangle are discarded, exactly like clipping to a known ROI).
 *   4. Negative-space mask: any grid cell whose max return height is
 *      meaningfully above the known flat-terrain height is "not terrain" -
 *      i.e. it's the silhouette carved out of the terrain scan by an
 *      object sitting on top of it.
 *   5. ClusterMask(): flood-fill (8-connectivity) the mask into blobs,
 *      then a radius-merge pass fuses any fragments sparse sampling left.
 *   6. Classify each blob as JACK vs BOULE using cluster footprint size
 *      and mean intensity (metal boules reflect much more than the
 *      wooden jack, exactly like real LiDAR intensity channels do).
 *   7. Rank boules by planar distance to the detected jack centroid.
 *
 * Controls:
 *   R          - randomize ball layout + re-scan
 *   1 / 2 / 3 / 4 or LEFT/RIGHT - step through the pipeline stages
 *   SPACE      - toggle ground-truth spheres (see the "blind" detection)
 *   Left-drag  - orbit camera, wheel - zoom
 */

#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------
#define FIELD_W        4.0f     // field width  (x)
#define FIELD_L        8.0f     // field length (y, "depth")
#define BOULE_RADIUS   0.0375f  // ~75mm diameter
#define JACK_RADIUS    0.015f   // ~30mm diameter
#define NUM_BOULES     6

#define GRID_RES       0.02f    // grid cell size in meters (must resolve the
                                 // jack's ~30mm footprint into several cells)
// NOTE: kept as literal integers (not computed from the floats above) so
// they remain true compile-time integer constants for array sizing.
// GRID_W = FIELD_W/GRID_RES + 1, GRID_L = FIELD_L/GRID_RES + 1
#define GRID_W         201
#define GRID_L         401

#define SCAN_AZ_STEPS  380
#define SCAN_EL_STEPS  220

#define NUM_SCANNERS   2
#define MAX_POINTS     (NUM_SCANNERS * SCAN_AZ_STEPS * SCAN_EL_STEPS)
#define MAX_CLUSTERS   64

// classification of a lidar return
typedef enum { HIT_NONE = 0, HIT_TERRAIN, HIT_BOULE, HIT_JACK, HIT_OUTSIDE } HitType;

typedef struct {
    Vector3 pos;
    float intensity;
    HitType type;
    int scannerId;
} LidarPoint;

typedef struct {
    float maxZ;        // highest return height seen in this cell
    float sumIntensity;
    int   count;
    bool  hasReturn;
} GridCell;

typedef struct {
    Vector2 centroid;   // field-space x,y
    float   avgIntensity;
    float   maxHeight;
    int     cellCount;
    bool    isJack;
    int     rank;       // 0 = jack itself / unranked, 1..N = closest..farthest boule
} Cluster;

// ---------------------------------------------------------------------
// Ground truth scene state
// ---------------------------------------------------------------------
static Vector3 boules[NUM_BOULES];
static Vector3 jack;
static Vector3 scannerPos[NUM_SCANNERS]; // 3m poles on opposite sides of the field

// scan results
static LidarPoint points[MAX_POINTS];
static int pointCount = 0;

// detection grid
static GridCell grid[GRID_L][GRID_W];
static bool objectMask[GRID_L][GRID_W]; // "negative space" mask
static int  clusterId[GRID_L][GRID_W];  // -1 = none

static Cluster clusters[MAX_CLUSTERS];
static int clusterCount = 0;
static int jackClusterIdx = -1;

// ---------------------------------------------------------------------
// Helpers: field-space <-> grid-space
// ---------------------------------------------------------------------
static inline bool InField(float x, float y) {
    return x >= 0.0f && x <= FIELD_W && y >= 0.0f && y <= FIELD_L;
}
static inline int GX(float x) { return (int)(x / GRID_RES); }
static inline int GY(float y) { return (int)(y / GRID_RES); }
static inline float CellX(int gx) { return (gx + 0.5f) * GRID_RES; }
static inline float CellY(int gy) { return (gy + 0.5f) * GRID_RES; }

// world coords: field occupies X in [0,FIELD_W], Z in [0,FIELD_L] (world Z = field "y"/depth), Y is up.
static inline Vector3 FieldToWorld(float fx, float fy, float z) {
    return (Vector3){ fx - FIELD_W * 0.5f, z, fy - FIELD_L * 0.5f };
}

// ---------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------
static void RandomizeScene(void) {
    jack.x = 0.6f + (float)GetRandomValue(0, 1000) / 1000.0f * (FIELD_W - 1.2f);
    jack.y = 0.6f + (float)GetRandomValue(0, 1000) / 1000.0f * (FIELD_L - 1.2f);
    jack.z = JACK_RADIUS;

    for (int i = 0; i < NUM_BOULES; i++) {
        bool ok = false;
        Vector2 p = {0};
        int tries = 0;
        while (!ok && tries < 200) {
            tries++;
            p.x = 0.3f + (float)GetRandomValue(0, 1000) / 1000.0f * (FIELD_W - 0.6f);
            p.y = 0.3f + (float)GetRandomValue(0, 1000) / 1000.0f * (FIELD_L - 0.6f);
            ok = true;
            float dj = Vector2Distance(p, (Vector2){jack.x, jack.y});
            if (dj < BOULE_RADIUS + JACK_RADIUS + 0.03f) { ok = false; continue; }
            for (int k = 0; k < i; k++) {
                float dk = Vector2Distance(p, (Vector2){boules[k].x, boules[k].y});
                if (dk < 2 * BOULE_RADIUS + 0.02f) { ok = false; break; }
            }
        }
        boules[i] = (Vector3){ p.x, p.y, BOULE_RADIUS };
    }
}

// ---------------------------------------------------------------------
// Ray / sphere intersection (sphere centers stored in field-space x,y,
// with .z used as radius-height, i.e. sphere center height = its .z field
// meaning "resting height" here we treat spheres as centered at
// (fx, fy, r) in field/world-up space). We do the actual math in the
// FIELD coordinate frame: x=field x, y=field y (depth), z=height (up).
// ---------------------------------------------------------------------
typedef struct { Vector3 o, d; } FRay; // field-space ray (x,y=depth,z=up)

static bool RaySphere(FRay r, Vector3 center, float radius, float *tHit) {
    Vector3 oc = Vector3Subtract(r.o, center);
    float b = Vector3DotProduct(oc, r.d);
    float c = Vector3DotProduct(oc, oc) - radius * radius;
    float disc = b * b - c;
    if (disc < 0) return false;
    float sq = sqrtf(disc);
    float t0 = -b - sq;
    float t1 = -b + sq;
    float t = (t0 > 1e-4f) ? t0 : t1;
    if (t <= 1e-4f) return false;
    *tHit = t;
    return true;
}

static bool RayGroundPlane(FRay r, float *tHit) {
    if (fabsf(r.d.z) < 1e-6f) return false;
    float t = (0.0f - r.o.z) / r.d.z;
    if (t <= 1e-4f) return false;
    *tHit = t;
    return true;
}

// ---------------------------------------------------------------------
// LiDAR scan: raster over azimuth/elevation from scannerPos (field space)
// ---------------------------------------------------------------------
static float SimIntensity(HitType t, unsigned int *seed) {
    // crude PRNG noise so intensity isn't perfectly flat (like real sensors)
    *seed = (*seed) * 1664525u + 1013904223u;
    float noise = ((*seed >> 8) % 1000) / 1000.0f * 0.08f - 0.04f;
    switch (t) {
        case HIT_BOULE:   return Clamp(0.90f + noise, 0.0f, 1.0f); // polished metal
        case HIT_JACK:    return Clamp(0.50f + noise, 0.0f, 1.0f); // wood
        case HIT_TERRAIN: return Clamp(0.20f + noise, 0.0f, 1.0f); // sand/gravel
        default:          return 0.0f;
    }
}

static void ScanFromPole(Vector3 pole, int scannerId, unsigned int *seed) {
    // Each pole looks toward the field center, angled down. Azimuth fov
    // and elevation band are sized so the fan reaches every corner of the
    // field from a 3m-high pole standing just outside one long edge.
    Vector2 fieldCenter = { FIELD_W * 0.5f, FIELD_L * 0.5f };
    Vector3 toCenter = { fieldCenter.x - pole.x, fieldCenter.y - pole.y, 0 - pole.z };
    float baseAz = atan2f(toCenter.y, toCenter.x);

    float azFovDeg = 150.0f; // wide enough to cover the full field length from a side pole
    float elMinDeg = -82.0f; // steeply down, toward the near edge
    float elMaxDeg = -18.0f; // shallower, out toward the far edge/corners

    for (int ai = 0; ai < SCAN_AZ_STEPS; ai++) {
        float azT = (float)ai / (SCAN_AZ_STEPS - 1) - 0.5f; // -0.5..0.5
        float az = baseAz + DEG2RAD * (azT * azFovDeg);
        for (int ei = 0; ei < SCAN_EL_STEPS; ei++) {
            float elT = (float)ei / (SCAN_EL_STEPS - 1);
            float elDeg = elMinDeg + elT * (elMaxDeg - elMinDeg);
            float el = DEG2RAD * elDeg;

            Vector3 dir = {
                cosf(el) * cosf(az),
                cosf(el) * sinf(az),
                sinf(el)
            };
            dir = Vector3Normalize(dir);

            FRay ray = { pole, dir };

            float bestT = 1e30f;
            HitType bestType = HIT_NONE;
            Vector3 bestPos = {0};

            for (int i = 0; i < NUM_BOULES; i++) {
                float t;
                if (RaySphere(ray, boules[i], BOULE_RADIUS, &t) && t < bestT) {
                    bestT = t; bestType = HIT_BOULE;
                    bestPos = Vector3Add(ray.o, Vector3Scale(ray.d, t));
                }
            }
            {
                float t;
                if (RaySphere(ray, jack, JACK_RADIUS, &t) && t < bestT) {
                    bestT = t; bestType = HIT_JACK;
                    bestPos = Vector3Add(ray.o, Vector3Scale(ray.d, t));
                }
            }
            {
                float t;
                if (RayGroundPlane(ray, &t) && t < bestT) {
                    Vector3 p = Vector3Add(ray.o, Vector3Scale(ray.d, t));
                    bestT = t;
                    bestPos = p;
                    bestType = InField(p.x, p.y) ? HIT_TERRAIN : HIT_OUTSIDE;
                }
            }

            if (bestType != HIT_NONE && pointCount < MAX_POINTS) {
                LidarPoint *lp = &points[pointCount++];
                lp->pos = bestPos;
                lp->type = bestType;
                lp->intensity = SimIntensity(bestType, seed);
                lp->scannerId = scannerId;
            }
        }
    }
}

static void LidarScan(void) {
    pointCount = 0;
    unsigned int seed = 12345u;
    for (int s = 0; s < NUM_SCANNERS; s++) {
        if (pointCount >= MAX_POINTS) break;
        ScanFromPole(scannerPos[s], s, &seed);
    }
}

// ---------------------------------------------------------------------
// Build top-down grid from raw returns, clipped to the field ROI.
// This is where "reflectivity of the balls" and geometry both feed in:
// we don't use material labels here at all (that would be cheating) -
// only height + intensity, exactly what a real sensor would give us.
// ---------------------------------------------------------------------
static void BuildGrid(void) {
    memset(grid, 0, sizeof(grid));
    for (int i = 0; i < pointCount; i++) {
        LidarPoint *lp = &points[i];
        if (lp->type == HIT_OUTSIDE) continue; // clip to field ROI
        if (!InField(lp->pos.x, lp->pos.y)) continue;
        int gx = GX(lp->pos.x), gy = GY(lp->pos.y);
        if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_L) continue;
        GridCell *c = &grid[gy][gx];
        if (!c->hasReturn || lp->pos.z > c->maxZ) c->maxZ = lp->pos.z;
        c->sumIntensity += lp->intensity;
        c->count++;
        c->hasReturn = true;
    }
}

// Negative-space mask: cells whose highest return sits meaningfully above
// the known flat terrain (z ~ 0) are "not terrain" -> object footprint.
#define HEIGHT_THRESHOLD 0.008f

static void BuildMask(void) {
    memset(objectMask, 0, sizeof(objectMask));
    for (int gy = 0; gy < GRID_L; gy++)
        for (int gx = 0; gx < GRID_W; gx++) {
            GridCell *c = &grid[gy][gx];
            if (c->hasReturn && c->maxZ > HEIGHT_THRESHOLD)
                objectMask[gy][gx] = true;
        }
}

// Flood-fill connected components over objectMask -> clusters[]
static void ClusterMask(void) {
    for (int gy = 0; gy < GRID_L; gy++)
        for (int gx = 0; gx < GRID_W; gx++)
            clusterId[gy][gx] = -1;

    clusterCount = 0;
    static int stackX[GRID_W * GRID_L];
    static int stackY[GRID_W * GRID_L];

    for (int gy = 0; gy < GRID_L; gy++) {
        for (int gx = 0; gx < GRID_W; gx++) {
            if (!objectMask[gy][gx] || clusterId[gy][gx] != -1) continue;
            if (clusterCount >= MAX_CLUSTERS) break;

            int id = clusterCount++;
            Cluster *cl = &clusters[id];
            memset(cl, 0, sizeof(Cluster));
            float sumX = 0, sumY = 0, sumI = 0, maxH = 0;
            int cnt = 0, sp = 0;

            stackX[sp] = gx; stackY[sp] = gy; sp++;
            clusterId[gy][gx] = id;

            while (sp > 0) {
                sp--;
                int cx = stackX[sp], cy = stackY[sp];
                GridCell *cell = &grid[cy][cx];
                float wx = CellX(cx), wy = CellY(cy);
                sumX += wx; sumY += wy;
                sumI += (cell->count > 0) ? (cell->sumIntensity / cell->count) : 0;
                if (cell->maxZ > maxH) maxH = cell->maxZ;
                cnt++;

                // 8-connectivity: bridges the small gaps that a sparse,
                // angularly-sampled point cloud leaves inside a blob.
                static const int dxs[8] = {1,-1,0,0, 1,1,-1,-1};
                static const int dys[8] = {0,0,1,-1, 1,-1,1,-1};
                for (int k = 0; k < 8; k++) {
                    int nx = cx + dxs[k], ny = cy + dys[k];
                    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_L) continue;
                    if (!objectMask[ny][nx] || clusterId[ny][nx] != -1) continue;
                    clusterId[ny][nx] = id;
                    stackX[sp] = nx; stackY[sp] = ny; sp++;
                }
            }

            cl->centroid = (Vector2){ sumX / cnt, sumY / cnt };
            cl->avgIntensity = sumI / cnt;
            cl->maxHeight = maxH;
            cl->cellCount = cnt;
        }
    }
}

// Sparse angular sampling can still split one physical sphere into a
// couple of adjacent islands even with 8-connectivity. Merge any clusters
// whose centroids are closer than roughly one ball radius - this is the
// same idea as a DBSCAN radius-merge / connected-component post-pass used
// on real point clouds.
#define MERGE_RADIUS (BOULE_RADIUS * 1.6f)

static void MergeCloseClusters(void) {
    bool merged = true;
    while (merged) {
        merged = false;
        for (int i = 0; i < clusterCount && !merged; i++) {
            for (int j = i + 1; j < clusterCount; j++) {
                if (Vector2Distance(clusters[i].centroid, clusters[j].centroid) > MERGE_RADIUS) continue;

                Cluster *a = &clusters[i], *b = &clusters[j];
                int totalCells = a->cellCount + b->cellCount;
                Vector2 newCentroid = {
                    (a->centroid.x * a->cellCount + b->centroid.x * b->cellCount) / totalCells,
                    (a->centroid.y * a->cellCount + b->centroid.y * b->cellCount) / totalCells
                };
                a->centroid = newCentroid;
                a->avgIntensity = (a->avgIntensity * a->cellCount + b->avgIntensity * b->cellCount) / totalCells;
                a->maxHeight = fmaxf(a->maxHeight, b->maxHeight);
                a->cellCount = totalCells;

                // remove b by swapping with last
                clusters[j] = clusters[clusterCount - 1];
                clusterCount--;
                merged = true;
                break;
            }
        }
    }
}

// Classify clusters (jack vs boule) using footprint size + intensity,
// then rank boules by distance to the detected jack.
static void ClassifyAndRank(void) {
    jackClusterIdx = -1;
    if (clusterCount == 0) return;

    // Expected footprint (in cells) for a sphere of radius r seen from
    // above is roughly pi*r^2 / cellArea.
    float cellArea = GRID_RES * GRID_RES;
    float expectedJackCells = (PI * JACK_RADIUS * JACK_RADIUS) / cellArea;
    float expectedBouleCells = (PI * BOULE_RADIUS * BOULE_RADIUS) / cellArea;

    // pick the cluster that best matches "small + medium intensity" as jack.
    float bestScore = 1e30f;
    for (int i = 0; i < clusterCount; i++) {
        Cluster *c = &clusters[i];
        float sizeErr = fabsf((float)c->cellCount - expectedJackCells) / expectedJackCells;
        float intensityErr = fabsf(c->avgIntensity - 0.5f);
        float score = sizeErr + 2.0f * intensityErr;
        if (c->cellCount < expectedBouleCells * 0.65f) { // clearly smaller than a boule
            if (score < bestScore) { bestScore = score; jackClusterIdx = i; }
        }
    }
    // fallback: if nothing looked jack-like, assume smallest cluster is jack
    if (jackClusterIdx == -1) {
        int smallest = 0;
        for (int i = 1; i < clusterCount; i++)
            if (clusters[i].cellCount < clusters[smallest].cellCount) smallest = i;
        jackClusterIdx = smallest;
    }

    for (int i = 0; i < clusterCount; i++) clusters[i].isJack = (i == jackClusterIdx);

    // rank boule clusters by distance to jack centroid
    Vector2 jc = clusters[jackClusterIdx].centroid;
    int order[MAX_CLUSTERS];
    int n = 0;
    for (int i = 0; i < clusterCount; i++) if (i != jackClusterIdx) order[n++] = i;

    for (int a = 0; a < n; a++)
        for (int b = a + 1; b < n; b++) {
            float da = Vector2Distance(clusters[order[a]].centroid, jc);
            float db = Vector2Distance(clusters[order[b]].centroid, jc);
            if (db < da) { int tmp = order[a]; order[a] = order[b]; order[b] = tmp; }
        }
    for (int r = 0; r < n; r++) clusters[order[r]].rank = r + 1;
}

static void RunFullPipeline(void) {
    LidarScan();
    BuildGrid();
    BuildMask();
    ClusterMask();
    MergeCloseClusters();
    ClassifyAndRank();
}

// ---------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------
static Color ColorForHit(HitType t) {
    switch (t) {
        case HIT_TERRAIN: return (Color){160, 130, 90, 255};
        case HIT_BOULE:   return (Color){210, 210, 220, 255};
        case HIT_JACK:    return (Color){230, 150, 60, 255};
        case HIT_OUTSIDE: return (Color){60, 60, 60, 120};
        default: return GRAY;
    }
}

typedef enum { STAGE_SCAN = 0, STAGE_MASK, STAGE_CLUSTERS, STAGE_RANKING, STAGE_COUNT } Stage;

static const char *StageTitle(Stage s) {
    switch (s) {
        case STAGE_SCAN:     return "STEP 1/4 - Raw LiDAR returns";
        case STAGE_MASK:     return "STEP 2/4 - Negative-space mask (terrain subtracted)";
        case STAGE_CLUSTERS: return "STEP 3/4 - Connected-component clustering";
        case STAGE_RANKING:  return "STEP 4/4 - Jack identified, boules ranked";
        default: return "";
    }
}
static const char *StageDesc(Stage s) {
    switch (s) {
        case STAGE_SCAN:     return "Points colored by material: brown=terrain, silver=boule, orange=jack";
        case STAGE_MASK:     return "Red cells = grid cells whose return sits above known flat-terrain height";
        case STAGE_CLUSTERS: return "Flood-filled blobs get a centroid (marker) - no labels assigned yet";
        case STAGE_RANKING:  return "Smallest/mid-reflectivity blob = jack (green). Boules ranked by distance to it";
        default: return "";
    }
}

int main(void) {
    const int screenW = 1280, screenH = 800;
    InitWindow(screenW, screenH, "Petanque LiDAR Prototype - raylib");
    SetTargetFPS(60);

    SetRandomSeed(GetTime() > 0 ? 42 : 42);
    // Two 3m poles, one just outside each long side of the field, aimed
    // down and across at the field - this is the "both sides" mount the
    // user asked for. Two viewpoints also fill in most of the shadowing
    // a single scanner would suffer when one ball blocks another.
    scannerPos[0] = (Vector3){ -0.6f,          FIELD_L * 0.5f, 3.0f };
    scannerPos[1] = (Vector3){ FIELD_W + 0.6f, FIELD_L * 0.5f, 3.0f };

    RandomizeScene();
    RunFullPipeline();

    // ---- custom orbit camera (spherical coords around field center) ----
    // NOTE: raylib's built-in CAMERA_ORBITAL mode only auto-rotates on a
    // timer and does not read the mouse at all, so we drive it by hand:
    // left-drag changes yaw/pitch, wheel changes distance.
    float camYaw = -0.9f, camPitch = 0.55f, camDist = 10.0f;
    Vector3 camTarget = { 0.0f, 0.5f, 0.0f };

    Camera3D camera = {0};
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    bool showGroundTruth = false;
    Stage stage = STAGE_RANKING;

    while (!WindowShouldClose()) {
        // ---- camera input ----
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 d = GetMouseDelta();
            camYaw   -= d.x * 0.005f;
            camPitch += d.y * 0.005f;
            camPitch = Clamp(camPitch, -1.4f, 1.4f);
        }
        camDist -= GetMouseWheelMove() * 0.8f;
        camDist = Clamp(camDist, 2.0f, 25.0f);

        camera.position.x = camTarget.x + camDist * cosf(camPitch) * cosf(camYaw);
        camera.position.z = camTarget.z + camDist * cosf(camPitch) * sinf(camYaw);
        camera.position.y = camTarget.y + camDist * sinf(camPitch);
        camera.target = camTarget;

        // ---- key input ----
        if (IsKeyPressed(KEY_R)) { RandomizeScene(); RunFullPipeline(); }
        if (IsKeyPressed(KEY_SPACE)) showGroundTruth = !showGroundTruth;
        if (IsKeyPressed(KEY_ONE))   stage = STAGE_SCAN;
        if (IsKeyPressed(KEY_TWO))   stage = STAGE_MASK;
        if (IsKeyPressed(KEY_THREE)) stage = STAGE_CLUSTERS;
        if (IsKeyPressed(KEY_FOUR))  stage = STAGE_RANKING;
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) stage = (stage < STAGE_RANKING) ? stage + 1 : stage;
        if (IsKeyPressed(KEY_LEFT)  || IsKeyPressed(KEY_A)) stage = (stage > STAGE_SCAN) ? stage - 1 : stage;

        BeginDrawing();
        ClearBackground((Color){24, 26, 30, 255});

        BeginMode3D(camera);

        // field boundary + ground
        Vector3 fieldWorldCenter = FieldToWorld(FIELD_W * 0.5f, FIELD_L * 0.5f, 0);
        DrawPlane(fieldWorldCenter, (Vector2){FIELD_W, FIELD_L}, (Color){110, 90, 60, 255});
        DrawGrid(20, 1.0f);

        // field outline
        {
            Vector3 c0 = FieldToWorld(0, 0, 0.01f);
            Vector3 c1 = FieldToWorld(FIELD_W, 0, 0.01f);
            Vector3 c2 = FieldToWorld(FIELD_W, FIELD_L, 0.01f);
            Vector3 c3 = FieldToWorld(0, FIELD_L, 0.01f);
            DrawLine3D(c0, c1, YELLOW); DrawLine3D(c1, c2, YELLOW);
            DrawLine3D(c2, c3, YELLOW); DrawLine3D(c3, c0, YELLOW);
        }

        // scanner poles
        for (int s = 0; s < NUM_SCANNERS; s++) {
            Vector3 base = FieldToWorld(scannerPos[s].x, scannerPos[s].y, 0);
            Vector3 top = FieldToWorld(scannerPos[s].x, scannerPos[s].y, scannerPos[s].z);
            DrawCylinderEx(base, top, 0.03f, 0.03f, 8, (Color){120,120,120,255});
            DrawSphere(top, 0.07f, RED);
        }

        // ground-truth objects (toggle-able, since a real system wouldn't have this)
        if (showGroundTruth) {
            for (int i = 0; i < NUM_BOULES; i++) {
                Vector3 w = FieldToWorld(boules[i].x, boules[i].y, boules[i].z);
                DrawSphereWires(w, BOULE_RADIUS, 8, 8, Fade(SKYBLUE, 0.5f));
            }
            Vector3 jw = FieldToWorld(jack.x, jack.y, jack.z);
            DrawSphereWires(jw, JACK_RADIUS, 8, 8, Fade(GREEN, 0.6f));
        }

        // ---- STEP 1: raw point cloud (always shown, dimmed once we move past it) ----
        {
            float alpha = (stage == STAGE_SCAN) ? 1.0f : 0.35f;
            for (int i = 0; i < pointCount; i++) {
                LidarPoint *lp = &points[i];
                if (lp->type == HIT_OUTSIDE) continue;
                Vector3 w = FieldToWorld(lp->pos.x, lp->pos.y, lp->pos.z);
                DrawPoint3D(w, Fade(ColorForHit(lp->type), alpha));
            }
        }

        // ---- STEP 2: negative-space mask ----
        if (stage >= STAGE_MASK) {
            float alpha = (stage == STAGE_MASK) ? 0.55f : 0.2f;
            for (int gy = 0; gy < GRID_L; gy++) {
                for (int gx = 0; gx < GRID_W; gx++) {
                    if (!objectMask[gy][gx]) continue;
                    Vector3 w = FieldToWorld(CellX(gx), CellY(gy), 0.015f);
                    DrawCube(w, GRID_RES*0.95f, 0.008f, GRID_RES*0.95f, Fade(RED, alpha));
                }
            }
        }

        // ---- STEP 3/4: cluster centroids ----
        if (stage >= STAGE_CLUSTERS) {
            for (int i = 0; i < clusterCount; i++) {
                Cluster *c = &clusters[i];
                bool knowRoles = (stage >= STAGE_RANKING);
                Color col = knowRoles ? (c->isJack ? LIME : GOLD) : SKYBLUE;
                Vector3 w = FieldToWorld(c->centroid.x, c->centroid.y, 0.28f);
                DrawSphere(w, 0.045f, col);
                Vector3 down = FieldToWorld(c->centroid.x, c->centroid.y, 0.0f);
                DrawLine3D(w, down, col);
            }
        }

        // ---- STEP 4: ranking lines from jack to each boule ----
        if (stage >= STAGE_RANKING && jackClusterIdx >= 0) {
            Vector2 jc = clusters[jackClusterIdx].centroid;
            Vector3 jw = FieldToWorld(jc.x, jc.y, 0.02f);
            for (int i = 0; i < clusterCount; i++) {
                Cluster *c = &clusters[i];
                if (c->isJack) continue;
                Vector3 bw = FieldToWorld(c->centroid.x, c->centroid.y, 0.02f);
                DrawLine3D(jw, bw, Fade(GOLD, 0.8f));
            }
        }

        EndMode3D();

        // ---------------- 2D overlay: stage banner + labels + ranking panel ----------------
        DrawRectangle(0, 0, screenW, 56, Fade(BLACK, 0.6f));
        DrawText(StageTitle(stage), 18, 8, 22, RAYWHITE);
        DrawText(StageDesc(stage), 18, 32, 15, LIGHTGRAY);

        // per-object labels, with distance-to-jack once ranking is known
        if (stage >= STAGE_CLUSTERS) {
            for (int i = 0; i < clusterCount; i++) {
                Cluster *c = &clusters[i];
                Vector3 w = FieldToWorld(c->centroid.x, c->centroid.y, 0.38f);
                Vector2 sp = GetWorldToScreen(w, camera);

                const char *label;
                Color col;
                if (stage >= STAGE_RANKING) {
                    if (c->isJack) { label = "JACK"; col = LIME; }
                    else {
                        float d = Vector2Distance(c->centroid, clusters[jackClusterIdx].centroid);
                        label = TextFormat("#%d  %.2fm", c->rank, d);
                        col = GOLD;
                    }
                } else {
                    label = TextFormat("obj %d", i + 1);
                    col = SKYBLUE;
                }
                DrawText(label, (int)sp.x - MeasureText(label, 18)/2, (int)sp.y - 22, 18, col);
            }
        }

        // ranking side panel (step 4 only)
        if (stage >= STAGE_RANKING && jackClusterIdx >= 0) {
            int panelH = 30 + 22 * (1 + (clusterCount > 0 ? clusterCount - 1 : 0));
            DrawRectangle(10, 64, 300, panelH, Fade(BLACK, 0.6f));
            int ty = 72;
            DrawText("Distance to jack (closest -> farthest):", 18, ty, 15, RAYWHITE); ty += 22;
            for (int r = 1; r <= clusterCount; r++) {
                for (int i = 0; i < clusterCount; i++) {
                    if (clusters[i].isJack || clusters[i].rank != r) continue;
                    float d = Vector2Distance(clusters[i].centroid, clusters[jackClusterIdx].centroid);
                    DrawText(TextFormat("#%d   %.3f m", r, d), 24, ty, 17, GOLD);
                    ty += 22;
                }
            }
        }

        DrawText("R: rescan   1-4 or LEFT/RIGHT: step   SPACE: ground truth   drag: orbit   wheel: zoom",
                 18, screenH - 24, 15, GRAY);
        if (showGroundTruth) DrawText("[ground truth ON]", screenW - 190, screenH - 24, 15, SKYBLUE);

        DrawFPS(screenW - 90, 64);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
