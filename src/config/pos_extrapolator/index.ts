import {
  PosExtrapolator,
  TagUseImuRotation,
} from "generated/thrift/gen-nodejs/pos_extrapolator_types";
import { MatrixUtil, VectorUtil } from "../util/math";
import { nav_x_config } from "./imu_config/navx";
import { kalman_filter } from "./kalman_filter_config";
import { message_config } from "./message_config";
import { swerve_odom_config } from "./odom_config/swerve_odom";
import { comp_lab } from "./tag_config/comp_lab";
import { reefscape_field } from "./tag_config/reefscape";

export const pose_extrapolator: PosExtrapolator = {
  message_config: message_config,
  camera_position_config: {
    front_left: {
      position: VectorUtil.fromArray([0.33, 0.33, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(-45),
    },
    front_right: {
      position: VectorUtil.fromArray([0.33, -0.33, 0.0]),
      rotation: MatrixUtil.buildRotationMatrixFromYaw(45),
    },
  },
  tag_position_config: reefscape_field,
  tag_confidence_threshold: 50,
  april_tag_discard_distance: 5,
  tag_use_imu_rotation: TagUseImuRotation.ALWAYS,
  enable_imu: false,
  enable_odom: false,
  enable_tags: false,
  odom_config: swerve_odom_config,
  imu_config: nav_x_config,
  kalman_filter_config: kalman_filter,
  time_s_between_position_sends: 0.025,
};

/*
front_right: {
  camera_robot_position: buildVector<number, 3>(0.33, -0.33, 0.0),
  camera_robot_direction: buildVector<number, 3>(
    Math.sqrt(2) / 2,
    -Math.sqrt(2) / 2,
    0.0
  ),
},
front_left: {
  camera_robot_position: buildVector<number, 3>(0.33, 0.33, 0.0),
  camera_robot_direction: buildVector<number, 3>(
    Math.sqrt(2) / 2,
    Math.sqrt(2) / 2,
    0.0
  ),
},
rear_right: {
  camera_robot_position: buildVector<number, 3>(-0.33, 0.33, 0.0),
  camera_robot_direction: buildVector<number, 3>(
    -Math.sqrt(2) / 2,
    Math.sqrt(2) / 2,
    0.0
  ),
},
rear_left: {
  camera_robot_position: buildVector<number, 3>(-0.33, -0.33, 0.0),
  camera_robot_direction: buildVector<number, 3>(
    -Math.sqrt(2) / 2,
    -Math.sqrt(2) / 2,
    0.0
  ),
},*/
