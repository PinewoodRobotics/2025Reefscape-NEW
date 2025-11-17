#!/bin/bash

source $HOME/.cargo/env
echo $MODULE_NAME
echo $PLATFORM_NAME
cd /work
cargo build --release --bin $MODULE_NAME --target-dir /target_build
ls -la /target_build/release/$MODULE_NAME
rm -f /work/target/release/$PLATFORM_NAME
mkdir -p /work/build/release/$PLATFORM_NAME
cp /target_build/release/$MODULE_NAME /work/build/release/$PLATFORM_NAME/$MODULE_NAME
ls -la /work/build/release/$PLATFORM_NAME/$MODULE_NAME
echo "Done compiling $MODULE_NAME for $PLATFORM_NAME finished!"