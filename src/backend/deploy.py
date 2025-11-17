from backend.deployment.util import (
    Module,
    ProtobufModule,
    PythonModule,
    RaspberryPi,
    RustModule,
    SystemType,
    ThriftModule,
    with_automatic_discovery,
    with_custom_backend_dir,
    with_preset_pi_addresses,
)


def get_modules() -> list[Module]:
    return [
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
        RustModule(
            project_root_folder_path="src/pathfinding",
            runnable_name="pathfinding",
            extra_run_args=[],
            equivalent_run_definition="pathfinding",
            build_for_platforms=[SystemType.PI5_BASE, SystemType.JETPACK_L4T_R35_2],
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
    # with_automatic_discovery(get_modules())

    with_preset_pi_addresses(
        [RaspberryPi(address="localhost", port=2222)], get_modules()
    )
