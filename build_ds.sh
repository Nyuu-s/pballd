#!/bin/bash

# Build raylib first (only needed once)
cd raylib/src
make clean
make PLATFORM=PLATFORM_DESKTOP RAYLIB_LIBTYPE=STATIC
cd ../..

# Compile the simulator
gcc -o lidar_sim lidar_sim.c -I./raylib/src -L./raylib/src -l:libraylib.a -lm -lGL -lpthread -ldl -lrt -lX11

echo "Build complete! Run ./lidar_sim"
