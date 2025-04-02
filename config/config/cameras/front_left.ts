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
    [453.79471998610825, 0, 323.102],
    [0, 453.71627459520215, 228.532],
    [0, 0, 1],
  ]),
  dist_coeff: buildVector<number, 5>(
    -0.23795732202168754,
    1.940403802169293,
    1.8219795137996125e-5,
    5.36105899753921e-5,
    -0.6071288444903038
  ),
};

export default front_left;
