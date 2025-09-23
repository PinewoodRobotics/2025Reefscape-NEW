import {
  KalmanFilterSensorType,
  type KalmanFilterConfig,
} from "generated/thrift/gen-nodejs/kalman_filter_types";
import { MatrixUtil, VectorUtil } from "../../util/math";

export const kalman_filter: KalmanFilterConfig = {
  state_vector: VectorUtil.fromArray([0.0, 0.0, 0.0, 0.0, 1.0, 0.0]), // [x, y, vx, vy, theta]
  time_step_initial: 0.1,
  state_transition_matrix: MatrixUtil.buildMatrix([
    [1.0, 0.0, 0.1, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.1, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
  ]),
  uncertainty_matrix: MatrixUtil.buildMatrix([
    [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
  ]),
  process_noise_matrix: MatrixUtil.buildMatrix([
    [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.01, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
  ]),
  dim_x_z: [6, 6],
  sensors: {
    [KalmanFilterSensorType.APRIL_TAG]: {
      april_tag: {
        measurement_conversion_matrix: MatrixUtil.buildMatrix([
          [1, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 1, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 1, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 1, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 1],
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrix([
          [5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 5.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]),
      },
    },
    [KalmanFilterSensorType.IMU]: {
      0: {
        measurement_conversion_matrix: MatrixUtil.buildMatrix([
          [1, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 1, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 1, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 1, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 1],
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrix([
          [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]),
      },
    },
    [KalmanFilterSensorType.ODOMETRY]: {
      odom: {
        measurement_conversion_matrix: MatrixUtil.buildMatrix([
          [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]),
        measurement_noise_matrix: MatrixUtil.buildMatrix([
          [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.2, 0.0],
          [0.0, 0.0, 0.0, 0.0, 0.0, 0.2],
        ]),
      },
    },
  },
};
