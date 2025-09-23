import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const front_right: CameraParameters = {
  pi_to_run_on: "agatha_king",
  name: "front_right",
  camera_path: "/dev/video-front-right",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 10,
  camera_matrix: MatrixUtil.buildMatrix([
    [682.908, 0.0, 404.37],
    [0.0, 682.84, 287.49],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray([
    0.044, -0.0502, 0.00082, -0.000109, -0.0479,
  ]),
  exposure_time: 5,
  camera_type: 0,
  compression_quality: 20,
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
