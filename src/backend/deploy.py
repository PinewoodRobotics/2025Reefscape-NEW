from backend.deployment.util import (
    CommonModule,
    ProtobufModule,
    PythonModule,
    RustModule,
    with_automatic_discovery,
    with_custom_backend_dir,
    with_exclusions_from_gitignore,
)


def get_modules() -> list[CommonModule]:
    return [
        PythonModule(
            local_root_folder_path="python/pos_extrapolator",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition="pos_extrapolator",
        ),
        PythonModule(
            local_root_folder_path="python/april",
            local_main_file_path="src/main.py",
            extra_run_args=[],
            equivalent_run_definition="april-server",
        ),
        RustModule(
            runnable_name="lidar-3d",
            extra_run_args=[],
            equivalent_run_definition="lidar-3d",
        ),
        ProtobufModule(
            project_root_folder_path="src/proto",
            extra_run_args=[],
            equivalent_run_definition="proto",
        ),
    ]


if __name__ == "__main__":
    with_custom_backend_dir("~/Documents/B.L.I.T.Z/backend")
    with_exclusions_from_gitignore(".gitignore")
    with_automatic_discovery(get_modules())
