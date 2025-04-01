import type { CameraParameters } from "../../schema/camera";
import { buildMatrixFromArray, buildVector } from "../util/math";

const front_left: CameraParameters = {
  pi_to_run_on: "agatha_king",
  name: "front_left",
  camera_path: "/dev/video-front-left",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 30,
  camera_matrix: buildMatrixFromArray<number, 3, 3>([
    [559.22044887, 0, 318.81940974],
    [0, 561.55018375, 225.56514441],
    [0, 0, 1],
  ]),
  dist_coeff: buildVector<number, 5>(
    0.05000835,
    -0.11020464,
    -0.00030042,
    0.00185564,
    0.0764982
  ),
};

export default front_left;
