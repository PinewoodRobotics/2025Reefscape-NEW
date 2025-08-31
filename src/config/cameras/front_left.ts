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
  width: 640,
  height: 480,
  max_fps: 10,
  camera_matrix: MatrixUtil.buildMatrix<3, 3>([
    [545.9558199647698, 0.0, 321.94949796294424],
    [0.0, 545.8780617956829, 229.2713390806784],
    [0.0, 0.0, 1.0],
  ]),
  dist_coeff: VectorUtil.fromArray<5>([
    8.71187465e-3, -8.77374006e-2, -1.62906725e-5, -1.47495281e-3,
    2.16825631e-2,
  ]),
  exposure_time: 5,
  camera_type: 0,
  compression_quality: 20,
};

export default front_left;
