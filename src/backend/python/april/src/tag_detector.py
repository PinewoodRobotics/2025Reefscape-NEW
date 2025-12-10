from dataclasses import dataclass

from cv2.typing import MatLike
from backend.generated.thrift.config.apriltag.ttypes import (
    AprilDetectionConfig,
    SpecialDetectorConfig,
)
import pyapriltags
from numpy.typing import NDArray
import numpy as np

from backend.python.april.src.deps_resolver import setup_cuda_tags_environment


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
    def use_cpu(
        cls,
        config: AprilDetectionConfig,
    ) -> "TagDetector":
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
        dist_coeffs: NDArray[np.float64],
        camera_matrix: NDArray[np.float64],
    ) -> "TagDetector":
        cuda_tags = setup_cuda_tags_environment(special_detector_config)

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        camera_matrix = cuda_tags.CameraMatrix(fx, cx, fy, cy)
        dist_coeffs = cuda_tags.DistCoeffs(
            dist_coeffs[0],
            dist_coeffs[1],
            dist_coeffs[2],
            dist_coeffs[3],
            dist_coeffs[4],
        )

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
