import time
import math

from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.thrift.config.common.ttypes import GenericMatrix, GenericVector
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorConfig,
    KalmanFilterSensorType,
)
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    ImuConfig,
    OdomConfig,
    OdometryPositionSource,
    PosExtrapolator,
    PosExtrapolatorMessageConfig,
    TagDisambiguationMode,
    TagUseImuRotation,
)
from backend.python.pos_extrapolator.data_prep import DataPreparerManager
from backend.python.pos_extrapolator.filters.extended_kalman_filter import (
    ExtendedKalmanFilterStrategy,
)
from backend.python.pos_extrapolator.position_extrapolator import PositionExtrapolator
from backend.python.pos_extrapolator.preparers.ImuDataPreparer import (
    ImuDataPreparerConfig,
)


def _eye(n: int) -> list[list[float]]:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def make_test_kalman_filter_config() -> KalmanFilterConfig:
    # 7D state: [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
    dim_x = 7
    dim_z = 5  # IMU provides [vx, vy, cos, sin, omega] in this configuration

    state_vector = GenericVector(values=[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], size=dim_x)
    F = GenericMatrix(values=_eye(dim_x), rows=dim_x, cols=dim_x)
    # Start very uncertain so the filter quickly trusts measurements in this test.
    P = GenericMatrix(
        values=[[100.0 if i == j else 0.0 for j in range(dim_x)] for i in range(dim_x)],
        rows=dim_x,
        cols=dim_x,
    )
    Q = GenericMatrix(
        values=[[0.01 if i == j else 0.0 for j in range(dim_x)] for i in range(dim_x)],
        rows=dim_x,
        cols=dim_x,
    )

    R = GenericMatrix(values=_eye(dim_z), rows=dim_z, cols=dim_z)
    H = GenericMatrix(values=_eye(dim_z), rows=dim_z, cols=dim_z)
    sensors = {
        KalmanFilterSensorType.IMU: {
            "0": KalmanFilterSensorConfig(
                measurement_noise_matrix=R,
                measurement_conversion_matrix=H,
            )
        }
    }

    return KalmanFilterConfig(
        state_vector=state_vector,
        state_transition_matrix=F,
        uncertainty_matrix=P,
        process_noise_matrix=Q,
        time_step_initial=0.05,
        sensors=sensors,
        dim_x_z=[dim_x, dim_z],
    )


def make_test_pos_extrapolator_config() -> PosExtrapolator:
    message_config = PosExtrapolatorMessageConfig(
        post_tag_input_topic="test/post_tag_input",
        post_odometry_input_topic="test/post_odom_input",
        post_imu_input_topic="test/post_imu_input",
        post_robot_position_output_topic="test/post_robot_position",
    )

    april_tag_config = AprilTagConfig(
        tag_position_config={},
        tag_disambiguation_mode=TagDisambiguationMode.NONE,
        camera_position_config={},
        tag_use_imu_rotation=TagUseImuRotation.NEVER,
        disambiguation_time_window_s=0.1,
    )

    odom_config = OdomConfig(
        position_source=OdometryPositionSource.DONT_USE, use_rotation=False
    )

    imu_config = {
        "0": ImuConfig(
            use_rotation=True,
            use_position=False,
            use_velocity=True,
        )
    }

    return PosExtrapolator(
        message_config=message_config,
        enable_imu=True,
        enable_odom=False,
        enable_tags=False,
        april_tag_config=april_tag_config,
        odom_config=odom_config,
        imu_config=imu_config,
        kalman_filter_config=make_test_kalman_filter_config(),
        future_position_prediction_margin_s=0.0,
    )


def initialize(config: PosExtrapolator):
    DataPreparerManager.set_config(ImuData, ImuDataPreparerConfig(config.imu_config))

    return PositionExtrapolator(
        config,
        ExtendedKalmanFilterStrategy(config.kalman_filter_config),
        DataPreparerManager(),
    )


def get_sample_imu_data():
    imu_data = ImuData()
    imu_data.velocity.x = 1.0
    imu_data.velocity.y = 2.0
    imu_data.velocity.z = 3.0
    imu_data.position.direction.x = math.cos(math.pi / 4)
    imu_data.position.direction.y = math.sin(math.pi / 4)

    return imu_data


def test_pose_extrapolator():
    position_extrapolator = initialize(make_test_pos_extrapolator_config())

    position_extrapolator.insert_sensor_data(get_sample_imu_data(), "0")
    time.sleep(1)
    position_extrapolator.insert_sensor_data(get_sample_imu_data(), "0")

    pos = position_extrapolator.get_robot_position()

    assert abs(float(pos.position_2d.position.x) - 1.0) < 0.1
    assert abs(float(pos.position_2d.position.y) - 2.0) < 0.1
    assert abs(float(pos.position_2d.direction.x) - math.cos(math.pi / 4)) < 0.1
    assert abs(float(pos.position_2d.direction.y) - math.sin(math.pi / 4)) < 0.1
