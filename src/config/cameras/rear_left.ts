import {
  CameraType,
  type CameraParameters,
} from "generated/thrift/gen-nodejs/camera_types";
import { MatrixUtil, VectorUtil } from "../util/math";

const rear_left: CameraParameters = {
  pi_to_run_on: "donnager",
  name: "rear_left",
  camera_path: "/dev/video4",
  flags: 0,
  width: 640,
  height: 480,
  max_fps: 30,
  camera_matrix: MatrixUtil.buildMatrix<3, 3>([
    [553.46572857, 0, 330.99059141],
    [0, 555.42286474, 242.75174591],
    [0, 0, 1],
  ]),
  dist_coeff: VectorUtil.fromArray<5>([
    0.04384078, -0.09394816, -0.0014493, 0.00057781, 0.07097694,
  ]),
  exposure_time: 5,
  camera_type: CameraType.OV2311,
};

export default rear_left;
