#!/bin/bash
set -e

source $HOME/.cargo/env
echo $MODULE_NAME
echo $PLATFORM_NAME
C_LIB_VERSION=$(ldd --version | head -n1 | awk '{print $NF}')
echo "C_LIB_VERSION: $C_LIB_VERSION"
cd /work

export CARGO_HOME=/work/target/$C_LIB_VERSION/$PLATFORM_NAME
mkdir -p $CARGO_HOME

export CARGO_TARGET_DIR=$CARGO_HOME
cargo build --release --bin $MODULE_NAME

ls -la $CARGO_HOME/release/$MODULE_NAME

mkdir -p /work/build/rust/release/$C_LIB_VERSION/$PLATFORM_NAME
cp $CARGO_HOME/release/$MODULE_NAME /work/build/rust/release/$C_LIB_VERSION/$PLATFORM_NAME/$MODULE_NAME
ls -la /work/build/rust/release/$C_LIB_VERSION/$PLATFORM_NAME/$MODULE_NAME

echo "Done compiling $MODULE_NAME for $C_LIB_VERSION $PLATFORM_NAME finished!"