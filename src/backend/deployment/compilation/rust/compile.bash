#!/bin/bash

source $HOME/.cargo/env
echo $MODULE_NAME
echo $PLATFORM_NAME
cd /work

export CARGO_HOME=/work/target/$PLATFORM_NAME
mkdir -p /work/target/$PLATFORM_NAME

export CARGO_TARGET_DIR=$CARGO_HOME
cargo build --release --bin $MODULE_NAME

ls -la /work/target/$PLATFORM_NAME/release/$MODULE_NAME

mkdir -p /work/build/release/$PLATFORM_NAME
cp /work/target/$PLATFORM_NAME/release/$MODULE_NAME /work/build/release/$PLATFORM_NAME/$MODULE_NAME
ls -la /work/build/release/$PLATFORM_NAME/$MODULE_NAME

echo "Done compiling $MODULE_NAME for $PLATFORM_NAME finished!"