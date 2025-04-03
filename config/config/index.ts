import type Config from "../schema";
import { pose_extrapolator } from "./pos_extrapolator";
import { autobahn_config } from "./autobahn";
import { april_tag_detection_config } from "./april_tags_detection";
import { kalman_filter } from "./pos_extrapolator/kalman_filter_config";
import { home_1 } from "./tag_config/home_1";
import { middle_campus } from "./tag_config/middle_campus";
import { physics_room } from "./tag_config/physics_room";
import { buildMatrixFromArray, buildVector } from "./util/math";
import front_left from "./cameras/front_left";
import front_right from "./cameras/front_right";
import rear_left from "./cameras/rear_left";
import rear_right from "./cameras/rear_right";

const config: Config = {
  pos_extrapolator: pose_extrapolator,
  autobahn: autobahn_config,
  cameras: [front_left, front_right],
  april_detection: april_tag_detection_config,
  logger: {
    enabled: false,
    profiler: {
      activated: false,
      profile_functions: false,
      time_functions: false,
      output_file: "profiler.json",
    },
    level: "DEBUG",
  },
  image_recognition: {
    model: "",
    device: "cpu",
    mode: "detection",
    training: {
      imgsz: 0,
      epochs: 0,
      batch_size: 0,
    },
    detection: {
      image_input_topic: "",
      image_output_topic: "",
      conf_threshold: 0,
      iou_threshold: 0,
      batch_size: 0,
    },
    message_config: {
      image_input_topic: "",
      inference_output_topic: "",
    },
    throwaway_time_ms: 0,
  },
  lidar_configs: [
    {
      pi_to_run_on: "pi-1",
      port: "/dev/ttyUSB0",
      baudrate: 115200,
      name: "lidar-1",
      is_2d: false,
      min_distance_meters: 0.1,
      max_distance_meters: 10.0,
      position_in_robot: buildVector<number, 3>(0.0, 0.0, 0.0),
      rotation_in_robot: buildVector<number, 3>(0.0, 0.0, 0.0),
    },
  ],
  watchdog: {
    send_stats: true,
    stats_interval_seconds: 1,
    stats_publish_topic: "stats/publish",
  },
};

export default config;
