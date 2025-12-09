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

ls -la

RESULT_PATH=/work/build/release/$C_LIB_VERSION/$PLATFORM_NAME/cpp/$MODULE_NAME

mkdir -p $RESULT_PATH

# Copy all shared libraries (.so) preserving subfolder structure (do NOT copy empty subfolders), ignoring CMakeFiles or any build folder
find . -type f -name "*.so" ! -path './.*' ! -path '*/CMakeFiles/*' ! -path '*/build/*' | while read -r file; do
    target="$RESULT_PATH/${file#./}"
    mkdir -p "$(dirname "$target")"
    cp "$file" "$target"
done

# Copy all static libraries (.a) preserving subfolder structure (do NOT copy empty subfolders), ignoring CMakeFiles or any build folder
find . -type f -name "*.a" ! -path './.*' ! -path '*/CMakeFiles/*' ! -path '*/build/*' | while read -r file; do
    target="$RESULT_PATH/${file#./}"
    mkdir -p "$(dirname "$target")"
    cp "$file" "$target"
done

# Copy all executables (not .so/.a/.o/.h/.cpp/.c/.txt/.md) preserving subfolder structure (do NOT copy empty subfolders), ignoring CMakeFiles or any build folder
find . -type f -perm -u=x ! -name "*.so" ! -name "*.a" ! -name "*.o" ! -name "*.h" ! -name "*.cpp" ! -name "*.c" ! -name "*.txt" ! -name "*.md" ! -path './.*' ! -path '*/CMakeFiles/*' ! -path '*/build/*' | while read -r file; do
    target="$RESULT_PATH/${file#./}"
    mkdir -p "$(dirname "$target")"
    cp "$file" "$target"
done