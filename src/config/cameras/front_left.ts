import {
  CameraParameters,
  CameraType,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const front_left: CameraParameters = {
  pi_to_run_on: "agatha_king",
  name: "front_left",
  camera_path: "/dev/video-front-left",
  flags: 0,
  width: 800,
  height: 600,
  max_fps: 100,
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
};

export default front_left;
