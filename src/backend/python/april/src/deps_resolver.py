import importlib
import os
import sys
from types import ModuleType
import importlib.util
from backend.generated.thrift.config.apriltag.ttypes import SpecialDetectorConfig


def setup_shared_library_python_extension(
    *,
    module_name: str,
    py_lib_searchpath: str,
    module_basename: str | None = None,
) -> ModuleType:
    module_basename = module_basename if module_basename else module_name

    module_parent = str(os.path.dirname(str(py_lib_searchpath)))
    if module_parent not in sys.path:
        sys.path.insert(0, module_parent)

    module_path = os.path.join(str(py_lib_searchpath), module_basename)
    extension_file: str | None = None
    dir_path = os.path.dirname(module_path)
    base_stem = os.path.basename(module_path)
    if os.path.isdir(dir_path):
        for fname in os.listdir(dir_path):
            if (
                fname.startswith(base_stem)
                and (fname.endswith(".so") or fname.endswith(".pyd"))
                and os.path.isfile(os.path.join(dir_path, fname))
            ):
                extension_file = os.path.join(dir_path, fname)
                break

    spec = importlib.util.spec_from_file_location(module_name, extension_file)
    if spec is None or spec.loader is None:
        raise ImportError(
            f"Failed to create spec for {module_name} from {extension_file}"
        )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def setup_cuda_tags_environment(
    special_detector_config: SpecialDetectorConfig,
) -> ModuleType:
    """
    Legacy wrapper for cuda_tags setup. Use setup_shared_library_python_extension directly for new code.
    """
    return setup_shared_library_python_extension(
        module_name="cuda_tags",
        py_lib_searchpath=special_detector_config.py_lib_searchpath,
        module_basename="cuda_tags",
    )
