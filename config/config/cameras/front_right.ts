import type { CameraParameters } from "../../schema/camera";
import { buildMatrixFromArray, buildVector } from "../util/math";

const front_right: CameraParameters = {
  pi_to_run_on: "agatha_king",
  name: "front_right",
  camera_path: "/dev/video-front-right",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 30,
  camera_matrix: buildMatrixFromArray<number, 3, 3>([
    [544.433, 0, 323.102],
    [0, 544.35, 228.532],
    [0, 0, 1],
  ]),
  dist_coeff: buildVector<number, 5>(
    0.05091232,
    -0.11548729,
    0.00114166,
    -0.00130648,
    0.03452092
  ),
};

export default front_right;

/*
  camera_matrix: buildMatrixFromArray<number, 3, 3>([
    [512.71702172, 0, 410.11248372],
    [0, 515.49273551, 322.10292672],
    [0, 0, 1],
  ]),*/

/*
  [
    [544.433, 0, 323.102],
    [0, 544.35, 228.532],
    [0, 0, 1],
  ]*/
