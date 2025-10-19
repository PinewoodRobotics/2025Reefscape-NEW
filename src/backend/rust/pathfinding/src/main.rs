use std::sync::{Arc, LazyLock, RwLock};

use autobahn_client::{
    autobahn::{Address, Autobahn},
    server_function,
};
use clap::Parser;
use common_core::{
    config::from_uncertainty_config,
    device_info::load_system_config,
    proto::{
        pathfind::{grid::Grid, PathfindRequest, PathfindResult},
        sensor::{general_sensor_data::Data, lidar_data, GeneralSensorData},
        util::{robot_position::Position, RobotPosition},
    },
};
use nalgebra::{Isometry2, Vector2};
use prost::{bytes::Bytes, Message};
use tokio::time;

use crate::grid::{Grid2d, ProtobufSerializable};

pub mod astar;
pub mod grid;
pub mod math;

static GRID: LazyLock<Arc<RwLock<Option<Grid2d>>>> = LazyLock::new(|| Arc::new(RwLock::new(None)));

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    config: Option<String>,
}

#[server_function]
async fn pathfind(req: PathfindRequest) -> PathfindResult {
    assert!(req.start.is_some());
    assert!(req.goal.is_some());

    let start = req.start.unwrap();
    let goal = req.goal.unwrap();

    let grid = GRID.read().unwrap();
    if grid.is_none() {
        eprintln!("No grid loaded");
        return PathfindResult { path: vec![] };
    }

    let grid = grid.as_ref().unwrap();

    let start = Vector2::new(start.x as usize, start.y as usize);
    let goal = Vector2::new(goal.x as usize, goal.y as usize);

    let mut path = grid.astar(start, goal);

    if req.optimize_path {
        path = grid.optimize_path(path);
    }

    path.serialize()
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let global_position: Arc<RwLock<Option<Isometry2<f32>>>> = Arc::new(RwLock::new(None));

    let args = Args::parse();

    let system_config = load_system_config()?;
    let config = from_uncertainty_config(args.config.as_deref())?;

    let autobahn = Autobahn::new_default(Address::new(
        system_config.autobahn.host,
        system_config.autobahn.port,
    ));
    autobahn.begin().await?;

    let pathfind_config = config.pathfinding;

    let grid = Grid2d::from_map_data(pathfind_config.map_data);
    GRID.write().unwrap().replace(grid);

    if pathfind_config.lidar_config.use_lidar && pathfind_config.global_pose_pub_topic.is_some() {
        let global_pose_pub_topic = pathfind_config.global_pose_pub_topic.unwrap();

        let global_position_clone = global_position.clone();
        autobahn
            .subscribe(global_pose_pub_topic.as_str(), move |msg| {
                let global_position_clone = global_position_clone.clone();
                async move {
                    let updated_global_position = RobotPosition::decode(Bytes::from(msg)).unwrap();
                    let updated_global_position = match updated_global_position.position {
                        Some(Position::Position2d(position)) => Some(position.into()),
                        Some(Position::Position3d(_)) => None,
                        None => None,
                    };

                    if updated_global_position.is_none() {
                        return;
                    }

                    global_position_clone
                        .write()
                        .unwrap()
                        .replace(updated_global_position.unwrap());
                }
            })
            .await?;

        let unit_conversion = pathfind_config.lidar_config.unit_conversion.clone();
        autobahn
            .subscribe(
                pathfind_config.lidar_config.lidar_pub_topic.as_str(),
                move |msg| {
                    let global_position_clone = global_position.clone();
                    let grid = GRID.clone();
                    let unit_conversion = unit_conversion.clone();

                    async move {
                        let global_position_clone = global_position_clone.clone();
                        if global_position_clone.read().unwrap().is_none() {
                            return;
                        }

                        let general_sensor_data =
                            GeneralSensorData::decode(Bytes::from(msg)).unwrap();
                        let data = general_sensor_data.data.unwrap();
                        assert!(matches!(data, Data::Lidar(_)));
                        let data = match data {
                            Data::Lidar(lidar_data) => lidar_data,
                            _ => unreachable!(),
                        };

                        let lidar_data = data.data.unwrap();
                        assert!(matches!(lidar_data, lidar_data::Data::PointCloud3d(_)));

                        let _lidar_data = match lidar_data {
                            lidar_data::Data::PointCloud3d(point_cloud_3d) => point_cloud_3d,
                            _ => unreachable!(),
                        };

                        let points_robot_frame = _lidar_data.ranges;
                        let points_global_frame =
                            math::transform_points_to_global_frame_and_convert_units(
                                &points_robot_frame,
                                &global_position_clone.read().unwrap().as_ref().unwrap(),
                                unit_conversion,
                            );

                        let mut grid = grid.write().unwrap();
                        if grid.is_none() {
                            return;
                        }

                        let grid = grid.as_mut().unwrap();
                        grid.clear_temporary_all();
                        grid.add_temporary(points_global_frame);
                    }
                },
            )
            .await?;
    }

    let sigint = tokio::signal::ctrl_c();
    tokio::pin!(sigint);

    loop {
        tokio::select! {
            _ = &mut sigint => {
                break;
            }
            _ = time::sleep(time::Duration::from_millis(100)) => {
                let grid = GRID.read().unwrap();
                let grid = grid.as_ref().unwrap();

                let grid_data = grid.serialize();
                let mut buf = Vec::new();
                Grid::Grid2d(grid_data).encode(&mut buf);
                autobahn
                    .publish(pathfind_config.map_pub_topic.as_str(), buf)
                    .await?;
            }
        }
    }

    return Ok(());
}
