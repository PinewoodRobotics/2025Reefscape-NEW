#!/bin/bash
set -e

echo $MODULE_NAME
echo $PLATFORM_NAME
echo $PROJECT_PATH
C_LIB_VERSION=$(ldd --version | head -n1 | awk '{print $NF}')
echo "C_LIB_VERSION: $C_LIB_VERSION"
cd /work/src/backend/$PROJECT_PATH

echo $BUILD_CMD
eval "$BUILD_CMD"

cd build
ls -la

mkdir -p /work/build/cpp/release/$C_LIB_VERSION/$PLATFORM_NAME

# Copy all shared libraries (.so)
find . -maxdepth 1 -type f -name "*.so" -exec cp {} /work/build/cpp/release/$C_LIB_VERSION/$PLATFORM_NAME/ \;

# Copy all static libraries (.a)
find . -maxdepth 1 -type f -name "*.a" -exec cp {} /work/build/cpp/release/$C_LIB_VERSION/$PLATFORM_NAME/ \;

# Copy all executables (files with execute permission, not ending in .so/.a/.o/.h/.cpp/.c/.txt/.md)
find . -maxdepth 1 -type f -perm -u=x ! -name "*.so" ! -name "*.a" ! -name "*.o" ! -name "*.h" ! -name "*.cpp" ! -name "*.c" ! -name "*.txt" ! -name "*.md" -exec cp {} /work/build/cpp/release/$C_LIB_VERSION/$PLATFORM_NAME/ \;