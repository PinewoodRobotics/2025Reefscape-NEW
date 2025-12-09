from typing import cast

from backend.deployment.compilation_util import CPPBuildConfig, CPPLibrary
from backend.deployment.system_types import SystemType
from backend.deployment.util import (
    CPPLibraryModule,
    RustModule,
    Module,
    ProtobufModule,
    PythonModule,
    RustModule,
    ThriftModule,
    with_automatic_discovery,
)


def get_modules() -> list[Module]:
    return [
        CPPLibraryModule(
            project_root_folder_path="src/cpp/CudaTags",
            build_for_platforms=[SystemType.PI5_BASE_PREBUILT],
            compilation_config=CPPBuildConfig.with_cmake(
                clean_build_dir=False,
                cmake_args=[
                    "-DCUDATAGS_BUILD_PYTHON=ON",
                ],
                libs=[
                    CPPLibrary(name="python3"),
                    CPPLibrary(name="python3-dev"),
                    CPPLibrary(name="python-is-python3"),
                    CPPLibrary(name="python3-numpy"),
                    CPPLibrary(name="python3-pip"),
                    CPPLibrary(name="python3-distutils"),
                    CPPLibrary(name="pybind11-dev"),
                    CPPLibrary(name="libopencv-dev"),
                    CPPLibrary(name="openjdk-11-jdk"),
                    CPPLibrary(name="default-jdk"),
                    CPPLibrary(name="cmake"),
                    CPPLibrary(name="ninja-build"),
                    CPPLibrary(name="pkg-config"),
                    CPPLibrary(name="git"),
                    CPPLibrary(name="build-essential"),
                    CPPLibrary(name="libssl-dev"),
                ],
                extra_docker_commands=[],
            ),
        ),
        PythonModule(
            local_root_folder_path="python/pos_extrapolator",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition="position-extrapolator",
        ),
        PythonModule(
            local_root_folder_path="python/april",
            local_main_file_path="src/main.py",
            extra_run_args=[],
            equivalent_run_definition="april-server",
        ),
        PythonModule(
            local_root_folder_path="python/image_recognition",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition="object-detector",
        ),
        PythonModule(
            local_root_folder_path="python/mc_server",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition="minecraft-server",
        ),
        RustModule(
            project_root_folder_path="src/pathfinding",
            runnable_name="pathfinding",
            extra_run_args=[],
            equivalent_run_definition="pathfinding",
            build_for_platforms=[SystemType.PI5_BASE_PREBUILT],
        ),
        RustModule(
            project_root_folder_path="src/lidar_3d",
            runnable_name="lidar-3d",
            extra_run_args=[],
            equivalent_run_definition="lidar-3d",
            build_for_platforms=[SystemType.PI5_BASE_PREBUILT],
        ),
        ProtobufModule(
            project_root_folder_path="src/proto",
            build_for_platforms=[],
        ),
        ThriftModule(
            project_root_folder_path="ThriftTsConfig/schema",
            build_for_platforms=[],
        ),
    ]


if __name__ == "__main__":
    # with_custom_backend_dir("~/Documents/B.L.I.T.Z/backend")
    with_automatic_discovery(get_modules())

    # with_preset_pi_addresses(
    #     [RaspberryPi(address="localhost", port=2222)], get_modules()
    # )
