use autobahn_client::autobahn::{Address, Autobahn};
use clap::Parser;
use common_core::config::from_uncertainty_config;
use common_core::device_info::{get_system_name, load_system_config};
use common_core::math::to_transformation_matrix_vec_matrix_f64;
use common_core::proto::sensor::general_sensor_data::Data;
use common_core::proto::sensor::LidarData;
use common_core::proto::sensor::{lidar_data, PointCloud3d};
use common_core::proto::sensor::{GeneralSensorData, SensorName};
use common_core::proto::util::Vector3;
use common_core::thrift::config::Config;
use common_core::thrift::lidar::LidarConfig;
use futures_util::StreamExt;
use prost::Message;
use unitree_lidar_l1_rust::lidar::reader::{LidarReader, LidarResult};

use crate::util::transform_point;

mod timed_point_map;
mod util;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    config: Option<String>,
}

fn get_lidar_config(config: &Config, current_pi: &str) -> Vec<(String, LidarConfig)> {
    let mut output_lidar_configs = Vec::new();
    for (key, lidar_config) in config.lidar_configs.iter() {
        if lidar_config.pi_to_run_on == current_pi {
            output_lidar_configs.push((key.clone(), lidar_config.clone()));
        }
    }

    output_lidar_configs
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let system_config = load_system_config()?;
    let current_pi = get_system_name()?;
    println!("Current pi: {}", current_pi);
    let config = from_uncertainty_config(args.config.as_deref())?;

    let autobahn = Autobahn::new_default(Address::new(
        system_config.autobahn.host,
        system_config.autobahn.port,
    ));

    autobahn.begin().await?;

    let lidar_configs = get_lidar_config(&config, &current_pi);

    let (lidar_name, config) = lidar_configs[0].clone(); // TODO: Handle multiple lidars
    let lidar_in_robot_transformation = to_transformation_matrix_vec_matrix_f64(
        config.position_in_robot.into(),
        config.rotation_in_robot.into(),
    );

    let mut reader = LidarReader::new_with_initialize(
        config.cloud_scan_num as u32,
        config.port,
        config.baudrate as u32,
        config.min_distance_meters.into(),
        config.max_distance_meters.into(),
        0.0,
        0.001,
        0.0,
    )?;
    reader.start_lidar()?;

    let mut reader = reader.into_stream();
    while let Some(result) = reader.next().await {
        match result {
            LidarResult::PointCloud(points) => {
                let points = points
                    .iter()
                    .map(|point| transform_point(point, &lidar_in_robot_transformation))
                    .map(|point| Vector3 {
                        x: point.x,
                        y: point.y,
                        z: point.z,
                    })
                    .collect::<Vec<_>>();

                let general_sensor_data = GeneralSensorData {
                    sensor_name: SensorName::Lidar as i32,
                    sensor_id: lidar_name.clone(),
                    timestamp: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_secs() as i64,
                    data: Some(Data::Lidar(LidarData {
                        data: Some(lidar_data::Data::PointCloud3d(PointCloud3d {
                            ranges: points,
                            lidar_id: lidar_name.clone(),
                        })),
                    })),
                    processing_time_ms: 0,
                };

                let _ = autobahn
                    .publish(
                        &format!("lidar/lidar3d/pointcloud/3d/robotframe"),
                        general_sensor_data.encode_to_vec(),
                    )
                    .await;
            }
            LidarResult::ImuReading(imu) => {}
        }
    }

    Ok(())
}
