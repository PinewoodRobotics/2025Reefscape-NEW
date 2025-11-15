ARG BUILD_DISTRO
FROM --platform=${TARGETPLATFORM} ${BUILD_DISTRO}

RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        ninja-build \
        ccache \
        pkg-config \
        git \
        g++ \
        clang \
        lldb \
        lcov \
        gcovr \
        valgrind \
        gdb \
        libssl-dev \
        libboost-all-dev \
        protobuf-compiler \
        thrift-compiler \
        wget \
        unzip \
        libgtest-dev \
        libgmock-dev && \
    cd /usr/src/gtest && cmake . && make && cp lib/*.a /usr/lib && \
    cd /usr/src/gmock && cmake . && make && cp lib/*.a /usr/lib

WORKDIR /work

ENTRYPOINT ["/bin/bash", "-c", "/work/src/backend/deployment/compilation/rust/compile.bash"]