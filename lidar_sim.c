#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <stdio.h>

#define MAX_POINTS 50000
#define MAX_OBJECTS 10
#define PI 3.14159265358979323846
#define GRID_SIZE 32   // Lower resolution for faster rendering

typedef struct {
    Vector3 position;
    int objectId;  // -2 = ground, -1 = unknown, 0 = target, >0 = balls
} LidarPoint;

typedef struct {
    Vector3 position;
    float radius;
    int id;
} Sphere;

typedef struct {
    Vector3 position;
    float azimuthMin, azimuthMax;
    float elevationMin, elevationMax;
    int raysPerScan;
    int pointCount;
    LidarPoint points[MAX_POINTS];
    int objectCount;
    Sphere objects[MAX_OBJECTS];
    float fieldWidth, fieldLength;
} LidarSimulator;

void initLidar(LidarSimulator* lidar, Vector3 pos, float azMin, float azMax,
               float elMin, float elMax, int rays, float w, float l) {
    lidar->position = pos;
    lidar->azimuthMin = azMin * PI / 180.0f;
    lidar->azimuthMax = azMax * PI / 180.0f;
    lidar->elevationMin = elMin * PI / 180.0f;
    lidar->elevationMax = elMax * PI / 180.0f;
    lidar->raysPerScan = rays;
    lidar->pointCount = 0;
    lidar->objectCount = 0;
    lidar->fieldWidth = w;
    lidar->fieldLength = l;
}

void addObject(LidarSimulator* lidar, Vector3 pos, float radius, int id) {
    if (lidar->objectCount < MAX_OBJECTS) {
        lidar->objects[lidar->objectCount].position = pos;
        lidar->objects[lidar->objectCount].radius = radius;
        lidar->objects[lidar->objectCount].id = id;
        lidar->objectCount++;
    }
}

float intersectSphere(Vector3 origin, Vector3 dir, Vector3 center, float radius) {
    Vector3 oc = { origin.x - center.x, origin.y - center.y, origin.z - center.z };
    float a = dir.x*dir.x + dir.y*dir.y + dir.z*dir.z;
    float b = 2.0f * (oc.x*dir.x + oc.y*dir.y + oc.z*dir.z);
    float c = oc.x*oc.x + oc.y*oc.y + oc.z*oc.z - radius*radius;
    float disc = b*b - 4*a*c;
    if (disc < 0) return -1.0f;
    float t1 = (-b - sqrtf(disc)) / (2*a);
    float t2 = (-b + sqrtf(disc)) / (2*a);
    if (t1 > 0.001f) return t1;
    if (t2 > 0.001f) return t2;
    return -1.0f;
}

float castRay(LidarSimulator* lidar, Vector3 origin, Vector3 dir) {
    float minT = INFINITY;

    if (dir.y < -0.0001f) {
        float t = -origin.y / dir.y;
        if (t > 0.001f) {
            Vector3 hit = { origin.x + dir.x*t, 0, origin.z + dir.z*t };
            if (hit.x >= -lidar->fieldWidth/2 && hit.x <= lidar->fieldWidth/2 &&
                hit.z >= -lidar->fieldLength/2 && hit.z <= lidar->fieldLength/2) {
                minT = t;
            }
        }
    }

    for (int i = 0; i < lidar->objectCount; i++) {
        float t = intersectSphere(origin, dir, lidar->objects[i].position, lidar->objects[i].radius);
        if (t > 0.001f && t < minT) minT = t;
    }
    return minT;
}

void performScan(LidarSimulator* lidar) {
    lidar->pointCount = 0;
    Vector3 origin = lidar->position;

    for (int i = 0; i < lidar->raysPerScan && lidar->pointCount < MAX_POINTS; i++) {
        float az = lidar->azimuthMin + ((float)rand() / RAND_MAX) * (lidar->azimuthMax - lidar->azimuthMin);
        float el = lidar->elevationMin + ((float)rand() / RAND_MAX) * (lidar->elevationMax - lidar->elevationMin);
        Vector3 dir = { cosf(el)*sinf(az), sinf(el), cosf(el)*cosf(az) };

        float t = castRay(lidar, origin, dir);
        if (t > 0.001f && t < 100.0f) {
            Vector3 hit = { origin.x + dir.x*t, origin.y + dir.y*t, origin.z + dir.z*t };
            LidarPoint pt = { hit, -1 };
            bool hitObject = false;
            for (int j = 0; j < lidar->objectCount; j++) {
                float d = sqrtf(powf(hit.x - lidar->objects[j].position.x, 2) +
                                powf(hit.y - lidar->objects[j].position.y, 2) +
                                powf(hit.z - lidar->objects[j].position.z, 2));
                if (d < lidar->objects[j].radius * 1.1f) {
                    pt.objectId = lidar->objects[j].id;
                    hitObject = true;
                    break;
                }
            }
            if (!hitObject && hit.y < 0.02f) {
                pt.objectId = -2;
            }
            lidar->points[lidar->pointCount++] = pt;
        }
    }
}

int filterGround(LidarPoint* src, int count, LidarPoint* dst, float threshold) {
    int out = 0;
    for (int i = 0; i < count; i++) {
        if (src[i].objectId >= 0 || src[i].position.y > threshold) {
            dst[out++] = src[i];
        }
    }
    return out;
}

void buildCoverageMap(LidarPoint* pts, int count, float w, float l, int gridSize, unsigned int* grid) {
    memset(grid, 0, gridSize * gridSize * sizeof(unsigned int));
    float halfW = w/2, halfL = l/2;
    for (int i = 0; i < count; i++) {
        float x = pts[i].position.x;
        float z = pts[i].position.z;
        if (x < -halfW || x > halfW || z < -halfL || z > halfL) continue;
        int ix = (int)((x + halfW) / w * gridSize);
        int iz = (int)((z + halfL) / l * gridSize);
        if (ix < 0 || ix >= gridSize || iz < 0 || iz >= gridSize) continue;
        grid[iz * gridSize + ix]++;
    }
}

int main() {
    InitWindow(1280, 720, "LiDAR Petanque Simulator");
    SetTargetFPS(60);
    srand((unsigned)time(NULL));

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 8.0f, 6.0f, 8.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float fieldW = 15.0f, fieldL = 4.0f;
    float sensorH = 3.0f;

    LidarSimulator s1, s2;
    int raysPerScan = 20000;
    initLidar(&s1, (Vector3){ 0, sensorH, -fieldL/2 }, -70, 70, -30, -85, raysPerScan, fieldW, fieldL);
    initLidar(&s2, (Vector3){ 0, sensorH, fieldL/2 }, 110, 250, -30, -85, raysPerScan, fieldW, fieldL);

    Vector3 targetPos = { 0.0f, 0.015f, 0.0f };
    addObject(&s1, targetPos, 0.015f, 0);
    addObject(&s2, targetPos, 0.015f, 0);

    Vector3 ballPositions[] = {
        { 1.2f, 0.037f, 0.5f },
        { -0.8f, 0.037f, -0.7f },
        { 0.5f, 0.037f, -1.2f },
        { -0.3f, 0.037f, 1.0f }
    };
    for (int i = 0; i < 4; i++) {
        addObject(&s1, ballPositions[i], 0.037f, i+1);
        addObject(&s2, ballPositions[i], 0.037f, i+1);
    }

    LidarPoint combined[MAX_POINTS * 2];
    LidarPoint filtered[MAX_POINTS * 2];
    int totalPoints = 0, filteredCount = 0;
    bool scanned = false, showFiltered = false, showCoverage = false;

    unsigned int coverageGrid[GRID_SIZE * GRID_SIZE];
    unsigned int maxCount = 1;

    char statusText[100] = "Press SPACE to scan";
    char tempBuffer[256];

    while (!WindowShouldClose()) {
        // ---- Input ----
        if (IsKeyPressed(KEY_SPACE)) {
            s1.raysPerScan = raysPerScan;
            s2.raysPerScan = raysPerScan;
            performScan(&s1);
            performScan(&s2);
            scanned = true;

            totalPoints = 0;
            for (int i = 0; i < s1.pointCount && totalPoints < MAX_POINTS*2; i++)
                combined[totalPoints++] = s1.points[i];
            for (int i = 0; i < s2.pointCount && totalPoints < MAX_POINTS*2; i++)
                combined[totalPoints++] = s2.points[i];

            filteredCount = filterGround(combined, totalPoints, filtered, 0.05f);
            sprintf(statusText, "Scanned! Points: %d, Filtered: %d", totalPoints, filteredCount);

            buildCoverageMap(combined, totalPoints, fieldW, fieldL, GRID_SIZE, coverageGrid);
            maxCount = 1;
            for (int i = 0; i < GRID_SIZE*GRID_SIZE; i++)
                if (coverageGrid[i] > maxCount) maxCount = coverageGrid[i];
        }

        if (IsKeyPressed(KEY_F)) {
            showFiltered = !showFiltered;
            sprintf(statusText, "Filter %s", showFiltered ? "ON" : "OFF");
        }
        if (IsKeyPressed(KEY_H)) {
            showCoverage = !showCoverage;
        }

        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
            raysPerScan = (raysPerScan < 50000) ? raysPerScan + 1000 : 50000;
            sprintf(statusText, "Rays: %d", raysPerScan);
        }
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
            raysPerScan = (raysPerScan > 1000) ? raysPerScan - 1000 : 1000;
            sprintf(statusText, "Rays: %d", raysPerScan);
        }

        // ---- Camera ----
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            float deltaX = GetMouseDelta().x;
            float deltaY = GetMouseDelta().y;
            Vector3 pivot = camera.target;
            Vector3 pos = camera.position;
            float dx = pos.x - pivot.x, dy = pos.y - pivot.y, dz = pos.z - pivot.z;
            float radius = sqrtf(dx*dx + dy*dy + dz*dz);
            float angleX = atan2(dx, dz) + deltaX * 0.01f;
            float angleY = atan2(dy, sqrt(dx*dx + dz*dz)) + deltaY * 0.01f;
            angleY = fmax(-1.5f, fmin(1.5f, angleY));
            camera.position.x = pivot.x + radius * cosf(angleY) * sinf(angleX);
            camera.position.z = pivot.z + radius * cosf(angleY) * cosf(angleX);
            camera.position.y = pivot.y + radius * sinf(angleY);
        }

        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            Vector3 dir = { camera.position.x - camera.target.x,
                            camera.position.y - camera.target.y,
                            camera.position.z - camera.target.z };
            float radius = sqrtf(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
            float newRadius = radius - wheel * 1.5f;
            newRadius = fmax(2.0f, fmin(30.0f, newRadius));
            float scale = newRadius / radius;
            camera.position.x = camera.target.x + dir.x * scale;
            camera.position.y = camera.target.y + dir.y * scale;
            camera.position.z = camera.target.z + dir.z * scale;
        }

        // ---- Rendering ----
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        DrawPlane((Vector3){0, 0, 0}, (Vector2){fieldW, fieldL}, LIGHTGRAY);
        DrawGrid(15, 1.0f);
        DrawLine3D((Vector3){-fieldW/2,0,-fieldL/2}, (Vector3){fieldW/2,0,-fieldL/2}, RED);
        DrawLine3D((Vector3){fieldW/2,0,-fieldL/2}, (Vector3){fieldW/2,0,fieldL/2}, RED);
        DrawLine3D((Vector3){fieldW/2,0,fieldL/2}, (Vector3){-fieldW/2,0,fieldL/2}, RED);
        DrawLine3D((Vector3){-fieldW/2,0,fieldL/2}, (Vector3){-fieldW/2,0,-fieldL/2}, RED);

        DrawSphere(s1.position, 0.1f, GREEN);
        DrawSphere(s2.position, 0.1f, GREEN);
        DrawLine3D((Vector3){s1.position.x,0,s1.position.z}, s1.position, GREEN);
        DrawLine3D((Vector3){s2.position.x,0,s2.position.z}, s2.position, GREEN);

        DrawSphere(targetPos, 0.015f, GOLD);
        Color ballColors[] = { BLUE, RED, GREEN, ORANGE };
        for (int i = 0; i < 4; i++)
            DrawSphere(ballPositions[i], 0.037f, ballColors[i]);

        if (scanned) {
            LidarPoint* pts = showFiltered ? filtered : combined;
            int cnt = showFiltered ? filteredCount : totalPoints;
            for (int i = 0; i < cnt; i++) {
                Color col = DARKGRAY;
                if (pts[i].objectId == 0) col = GOLD;
                else if (pts[i].objectId == 1) col = BLUE;
                else if (pts[i].objectId == 2) col = RED;
                else if (pts[i].objectId == 3) col = GREEN;
                else if (pts[i].objectId == 4) col = ORANGE;
                else if (pts[i].objectId == -2) col = DARKGRAY;
                else col = PURPLE;
                DrawSphere(pts[i].position, 0.02f, col);
            }
        }

        EndMode3D();

        // ---- 2D UI ----
        DrawText(statusText, 10, 10, 20, DARKGRAY);
        DrawText("SPACE: scan | F: filter ground | H: toggle coverage map", 10, 40, 16, DARKGRAY);
        sprintf(tempBuffer, "+/- : increase/decrease rays (current: %d)", raysPerScan);
        DrawText(tempBuffer, 10, 65, 16, DARKGRAY);
        sprintf(tempBuffer, "Points: %d (filtered: %d)", totalPoints, filteredCount);
        DrawText(tempBuffer, 10, 90, 16, DARKGRAY);
        sprintf(tempBuffer, "Filter: %s", showFiltered ? "ON" : "OFF");
        DrawText(tempBuffer, 10, 115, 16, showFiltered ? GREEN : RED);

        // Coverage map as a grid of rectangles (simpler, no textures)
        if (showCoverage && scanned && maxCount > 0) {
            int mapX = 10, mapY = 140, cellSize = 6;
            int mapW = GRID_SIZE * cellSize;
            int mapH = GRID_SIZE * cellSize;
            DrawRectangle(mapX-2, mapY-2, mapW+4, mapH+4, LIGHTGRAY);
            for (int iz = 0; iz < GRID_SIZE; iz++) {
                for (int ix = 0; ix < GRID_SIZE; ix++) {
                    float val = (float)coverageGrid[iz * GRID_SIZE + ix] / maxCount;
                    // Invert: bright = low count (occlusion)
                    unsigned char c = (unsigned char)((1.0f - val) * 255);
                    Color col = { c, c, c, 255 };
                    DrawRectangle(mapX + ix * cellSize, mapY + iz * cellSize, cellSize, cellSize, col);
                }
            }
            DrawText("Coverage map (bright = occlusion)", mapX, mapY + mapH + 5, 12, DARKGRAY);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
