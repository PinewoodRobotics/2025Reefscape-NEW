import time
import numpy as np
from cscore import CvSink, VideoSource
from numpy.typing import NDArray

from backend.generated.thrift.config.camera.ttypes import CameraParameters, CameraType
from backend.python.common.debug.logger import error, success
from backend.python.common.util.math import get_np_from_matrix, get_np_from_vector


class AbstractCaptureDevice:
    _registry: dict[CameraType, type["AbstractCaptureDevice"]] = {}

    def __init_subclass__(cls, type: CameraType, **kwargs):
        super().__init_subclass__(**kwargs)
        if type is not None:
            AbstractCaptureDevice._registry[type] = cls

    def __init__(
        self,
        camera_port: int | str,
        width: int,
        height: int,
        max_fps: float,
        camera_name: str,
        camera_matrix: NDArray[np.float64] = np.eye(3),
        dist_coeff: NDArray[np.float64] = np.zeros(5),
        hard_fps_limit: float | None = None,
        exposure_time: float | None = None,
    ):
        if isinstance(camera_port, str):
            try:
                camera_port = int(camera_port)
            except ValueError:
                pass

        self.port = camera_port
        self.camera_name = camera_name
        self.width = width
        self.height = height
        self.max_fps = max_fps
        self.hard_limit = hard_fps_limit
        self.camera_matrix = camera_matrix
        self.dist_coeff = dist_coeff
        self.frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.exposure_time = exposure_time

        self.camera: VideoSource | None = None
        self.sink: CvSink | None = None  # CameraServer.getVideo(self.camera)

        self._is_ready = False
        self._initialize_camera()
        self._last_ts = time.time()

    def get_name(self) -> str:
        return self.camera_name

    def _initialize_camera(self):
        self._configure_camera()
        if self.camera:
            self.camera.setConnectionStrategy(
                VideoSource.ConnectionStrategy.kConnectionKeepOpen
            )

            max_attempts = 5
            attempt = 0
            while attempt < max_attempts:
                if self.camera.isConnected() and self.sink is not None:
                    test_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    ts, _ = self.sink.grabFrame(test_frame)
                    if ts > 0:
                        self._is_ready = True
                        success(f"Camera successfully connected and initialized")
                        return
                attempt += 1
                error(
                    f"Waiting for camera to connect (attempt {attempt}/{max_attempts})..."
                )
                time.sleep(1.0)

            error(f"WARNING: Failed to initialize camera after {max_attempts} attempts")

    def get_frame(self) -> tuple[bool, NDArray[np.uint8] | None]:
        start = time.time()

        if self.sink is None or not self._is_ready:
            self._is_ready = False
            self._initialize_camera()
            return False, self.frame

        ts, self.frame = self.sink.grabFrame(self.frame)
        now = time.time()

        if ts == 0:
            error_msg = self.sink.getError()
            error(f"Error grabbing frame: {error_msg}")
            self._is_ready = False
            self._initialize_camera()
            self._last_ts = now
            return False, None

        if self.hard_limit:
            interval = 1.0 / self.hard_limit
            took = now - start
            if took < interval:
                time.sleep(interval - took)

        self._last_ts = time.time()

        return True, self.frame

    def release(self):
        self._is_ready = False
        self.sink = None
        self.camera = None

    def get_matrix(self) -> NDArray[np.float64]:
        return self.camera_matrix

    def get_dist_coeff(self) -> NDArray[np.float64]:
        return self.dist_coeff

    def _configure_camera(self):
        raise NotImplementedError()


def get_camera_capture_device(camera: CameraParameters) -> AbstractCaptureDevice:
    return AbstractCaptureDevice._registry[camera.camera_type](
        camera.camera_path,
        camera.width,
        camera.height,
        camera.max_fps,
        camera.name,
        get_np_from_matrix(camera.camera_matrix),
        get_np_from_vector(camera.dist_coeff),
        exposure_time=camera.exposure_time,
    )
