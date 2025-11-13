from backend.deployment.util import (
    Module,
    ProtobufModule,
    PythonModule,
    RustModule,
    ThriftModule,
    with_automatic_discovery,
    with_custom_backend_dir,
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
            runnable_name="pathfinding",
            cross_compile_target="aarch64-unknown-linux-gnu",
            extra_run_args=[],
            equivalent_run_definition="pathfinding",
        ),
        ProtobufModule(
            project_root_folder_path="src/proto",
        ),
        ThriftModule(
            project_root_folder_path="ThriftTsConfig/schema",
        ),
    ]


if __name__ == "__main__":
    with_automatic_discovery(get_modules())
