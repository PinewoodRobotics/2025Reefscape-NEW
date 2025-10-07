from backend.deployment.util import (
    CommonModule,
    PythonModule,
    with_automatic_discovery,
    with_custom_backend_dir,
)


def get_modules() -> list[CommonModule] | CommonModule:
    return [
        PythonModule(
            local_root_folder_path="python/pos_extrapolator",
            local_main_file_path="main.py",
            extra_run_args=[],
            equivalent_run_definition="pos_extrapolator",
        )
    ]


if __name__ == "__main__":
    with_custom_backend_dir("~/Documents/B.L.I.T.Z/backend")
    with_automatic_discovery()
