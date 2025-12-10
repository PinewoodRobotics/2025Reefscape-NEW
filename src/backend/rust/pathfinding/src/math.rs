use common_core::{proto::util::Vector3, thrift::common::UnitConversion};
use nalgebra::Isometry2;

pub fn transform_points_to_global_frame(
    points_robot_frame: &Vec<Vector3>,
    robot_in_global_frame: &Isometry2<f32>,
) -> Vec<nalgebra::Vector2<f32>> {
    points_robot_frame
        .iter()
        .map(|point| {
            let point2 = nalgebra::Vector2::new(point.x, point.y);
            let transformed = robot_in_global_frame * point2;
            nalgebra::Vector2::new(transformed.x, transformed.y)
        })
        .collect()
}

pub fn transform_points_to_global_frame_and_convert_units(
    points_robot_frame: &Vec<Vector3>,
    robot_in_global_frame: &Isometry2<f32>,
    conversion: UnitConversion,
) -> Vec<nalgebra::Vector2<usize>> {
    points_robot_frame
        .iter()
        .map(|point| {
            let point2 = nalgebra::Vector2::new(point.x, point.y);
            let transformed = robot_in_global_frame * point2;

            nalgebra::Vector2::new(
                (transformed.x * (*conversion.non_unit_to_unit as f32)) as usize,
                (transformed.y * (*conversion.non_unit_to_unit as f32)) as usize,
            )
        })
        .collect()
}

pub fn convert_to_map_units(
    points: &nalgebra::Vector2<f32>,
    map_to_meters: &UnitConversion,
) -> nalgebra::Vector2<usize> {
    nalgebra::Vector2::new(
        (points.x * (*map_to_meters.non_unit_to_unit as f32)) as usize,
        (points.y * (*map_to_meters.non_unit_to_unit as f32)) as usize,
    )
}
