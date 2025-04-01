import type { CameraParameters } from "../../schema/camera";
import { buildMatrixFromArray, buildVector } from "../util/math";

const rear_left: CameraParameters = {
  pi_to_run_on: "donnager",
  name: "rear_left",
  camera_path: "/dev/video-rear-left",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 30,
  camera_matrix: buildMatrixFromArray<number, 3, 3>([
    [553.46572857, 0, 330.99059141],
    [0, 555.42286474, 242.75174591],
    [0, 0, 1],
  ]),
  dist_coeff: buildVector<number, 5>(
    0.04384078,
    -0.09394816,
    -0.0014493,
    0.00057781,
    0.07097694
  ),
};

export default rear_left;
