#!/usr/bin/env bash
# Build script for the petanque LiDAR prototype.
# Works on Linux/macOS with raylib installed (or built from source).
#
# Option A - if raylib is installed system-wide (e.g. `brew install raylib`,
# or `sudo apt install libraylib-dev` where available):
#   gcc -O2 petanque_lidar.c -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -o petanque_lidar
#
# Option B - build raylib from source first (works anywhere with git+cmake):
set -e
if [ ! -d raylib ]; then
  git clone --depth 1 --branch 5.5 https://github.com/raysan5/raylib.git
fi
mkdir -p raylib/build
cd raylib/build
cmake .. -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DPLATFORM=Desktop
make -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"
cd ../..

gcc -O2 -Wall petanque_lidar.c \
  -I raylib/src \
  -L raylib/build/raylib \
  -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 \
  -o petanque_lidar

echo "Built ./petanque_lidar - run it with: ./petanque_lidar"
