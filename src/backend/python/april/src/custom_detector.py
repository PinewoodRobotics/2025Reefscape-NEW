from dataclasses import dataclass
import importlib
import os
import sys

from cv2.typing import MatLike
from backend.generated.thrift.config.apriltag.ttypes import (
    AprilDetectionConfig,
    SpecialDetectorConfig,
    SpecialDetectorType,
)
import pyapriltags
from numpy.typing import NDArray
import numpy as np
from pyapriltags.apriltags import Detection
from backend.python.common.debug.logger import info
from backend.python.common.util.system import get_system_name


@dataclass
class TagDetection:
    corners: NDArray[np.int32]
    tag_id: int
    hamming: int
    decision_margin: float
    homography: NDArray[np.float64]
    center: NDArray[np.float64]


class TagDetector:
    def __init__(self, detector, detector_type: str):
        self.detector = detector
        self.detector_type = detector_type

    @classmethod
    def use_cpu(cls, config: AprilDetectionConfig) -> "TagDetector":
        return cls(
            pyapriltags.Detector(
                families=str(config.family),
                nthreads=config.nthreads,
                quad_decimate=config.quad_decimate,
                quad_sigma=config.quad_sigma,
                refine_edges=config.refine_edges,
                decode_sharpening=config.decode_sharpening,
            ),
            "cpu_generic",
        )

    @classmethod
    def use_cuda_tags(
        cls,
        config: AprilDetectionConfig,
        special_detector_config: SpecialDetectorConfig,
        width: int,
        height: int,
    ) -> "TagDetector":
        python_lib_searchpath = special_detector_config.py_lib_searchpath
        cuda_tags_parent = str(os.path.dirname(str(python_lib_searchpath)))
        if cuda_tags_parent not in sys.path:
            sys.path.insert(0, cuda_tags_parent)

        # Add the cuda_tags directory to LD_LIBRARY_PATH FIRST (before other paths)
        # This ensures libraries from cuda_tags directory are found before /usr/local/lib/
        # The __init__.py will also add it, but we want it here first to ensure correct order
        cuda_tags_dir = str(python_lib_searchpath)
        if "LD_LIBRARY_PATH" in os.environ:
            if cuda_tags_dir not in os.environ["LD_LIBRARY_PATH"]:
                # Prepend to ensure this directory is checked first
                os.environ["LD_LIBRARY_PATH"] = (
                    f"{cuda_tags_dir}:{os.environ['LD_LIBRARY_PATH']}"
                )
        else:
            os.environ["LD_LIBRARY_PATH"] = cuda_tags_dir

        lib_searchpaths = special_detector_config.lib_searchpath
        for lib_searchpath in lib_searchpaths:
            if str(lib_searchpath) not in sys.path:
                sys.path.append(str(lib_searchpath))

            # Add additional library search paths AFTER cuda_tags directory
            cuda_tags_lib_path = str(lib_searchpath)
            if "LD_LIBRARY_PATH" in os.environ:
                if cuda_tags_lib_path not in os.environ["LD_LIBRARY_PATH"]:
                    os.environ["LD_LIBRARY_PATH"] = (
                        f"{os.environ['LD_LIBRARY_PATH']}:{cuda_tags_lib_path}"
                    )
            else:
                os.environ["LD_LIBRARY_PATH"] = cuda_tags_lib_path

        # Explicitly import the cuda_tags package using importlib to ensure __init__.py executes
        # This ensures the __init__.py in the cuda_tags directory is loaded and executed
        cuda_tags = importlib.import_module(
            "cuda_tags"
        )  # pyright: ignore[reportMissingImports]

        camera_matrix = cuda_tags.CameraMatrix(
            1000, 1000, 1000, 1000
        )  # rando values not used anywhere rn
        dist_coeffs = cuda_tags.DistCoeffs(0, 0, 0, 0, 0)
        tags_wrapper = cuda_tags.CudaTagsWrapper(
            cuda_tags.TagType.tag36h11,
            camera_matrix,
            dist_coeffs,
            config.nthreads,
            width,
            height,
        )

        detector_instance = cls(tags_wrapper, "cuda_tags")

        return detector_instance

    def detect(self, frame: NDArray[np.uint8] | MatLike) -> list[TagDetection]:
        return_value: list[TagDetection] = []
        if self.detector_type == "cuda_tags":
            detections = self.detector.process(frame)
            return_value.extend(
                [
                    TagDetection(
                        corners=np.array(detection.corners, dtype=np.int32),
                        tag_id=detection.id,
                        hamming=detection.hamming,
                        decision_margin=detection.decision_margin,
                        homography=np.array(detection.homography),
                        center=np.array(detection.center),
                    )
                    for detection in detections
                ]
            )
        elif self.detector_type == "cpu_generic":
            assert isinstance(self.detector, pyapriltags.Detector)
            detections = self.detector.detect(frame)
            return_value.extend(
                [
                    TagDetection(
                        corners=detection.corners.copy(),
                        tag_id=detection.tag_id,
                        hamming=detection.hamming,
                        decision_margin=detection.decision_margin,
                        homography=np.array(detection.homography).copy(),
                        center=np.array(detection.center).copy(),
                    )
                    for detection in detections
                ]
            )
        else:
            raise ValueError(f"Invalid detector type: {self.detector_type}")

        return return_value
