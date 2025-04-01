import { KalmanFilterConfig } from "../../../schema/pos-extrapolator/filters/kalman";
import { buildMatrixFromArray, buildVector } from "../../util/math";

export const kalman_filter: KalmanFilterConfig<5> = {
  state_vector: buildVector<number, 5>(0.0, 0.0, 0.0, 0.0, 0.0), // [x, y, vx, vy, theta]
  time_step_initial: 0.1,
  state_transition_matrix: buildMatrixFromArray<number, 5, 5>([
    [1.0, 0.0, 0.1, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.1, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0],
  ]),
  uncertainty_matrix: buildMatrixFromArray<number, 5, 5>([
    [1, 0.0, 0.0, 0.0, 0.0],
    [0.0, 1, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1],
  ]),

  process_noise_matrix: buildMatrixFromArray<number, 5, 5>([
    [10, 0.0, 0.0, 0.0, 0.0],
    [0.0, 10, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1],
  ]),
  dim_x_z: [5, 5],
  sensors: {
    "april-tag": {
      name: "april-tag",
      measurement_conversion_matrix: buildMatrixFromArray<number, 5, 5>([
        [1, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1],
      ]),
      measurement_noise_matrix: buildMatrixFromArray<number, 5, 5>([
        [0.3, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.3, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1000000, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1000000, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.3],
      ]),
    },
    odometry: {
      name: "odometry",
      measurement_conversion_matrix: buildMatrixFromArray<number, 5, 5>([
        [1, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1],
      ]),
      measurement_noise_matrix: buildMatrixFromArray<number, 5, 5>([
        [0.05, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.05, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.001, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.001, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.05],
      ]),
    },
    imu: {
      name: "imu",
      measurement_conversion_matrix: buildMatrixFromArray<number, 5, 5>([
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
      ]),
      measurement_noise_matrix: buildMatrixFromArray<number, 5, 5>([
        [0.5, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.5, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.01, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.01, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.2],
      ]),
    },
  },
};
