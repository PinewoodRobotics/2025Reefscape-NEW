use std::collections::HashSet;

use common_core::{proto::pathfind, thrift::common::MapData};
use nalgebra::Vector2;

#[derive(Clone)]
pub enum Cell {
    Passable,
    Obstacle,
    Temporary(Box<Cell>),
}

impl Cell {
    pub fn serialize(&self) -> pathfind::Cell {
        match self {
            Cell::Passable => pathfind::Cell::Passable,
            Cell::Obstacle => pathfind::Cell::Obstacle,
            Cell::Temporary(_) => pathfind::Cell::Temporary,
        }
    }
}

pub struct Grid2d {
    pub width: usize,
    pub height: usize,
    pub cells: Vec<Cell>,
    temp_cells: HashSet<usize>,
}

impl Grid2d {
    pub fn new(width: usize, height: usize, map_data: &[bool]) -> Self {
        Self {
            width,
            height,
            cells: map_data
                .iter()
                .map(|&x| if x { Cell::Passable } else { Cell::Obstacle })
                .collect(),
            temp_cells: HashSet::new(),
        }
    }

    pub fn from_map_data(map_data: MapData) -> Self {
        let width = map_data.map_size_x as usize;
        let height = map_data.map_size_y as usize;
        Self {
            width,
            height,
            cells: map_data
                .map_data
                .iter()
                .map(|&x| if x { Cell::Passable } else { Cell::Obstacle })
                .collect(),
            temp_cells: HashSet::new(),
        }
    }

    pub fn get_cell_location(&self, pos: Vector2<usize>) -> usize {
        pos.y * self.width + pos.x
    }

    pub fn in_bounds(&self, pos: Vector2<usize>) -> bool {
        pos.x < self.width && pos.y < self.height
    }

    pub fn is_passable(&self, pos: Vector2<usize>) -> bool {
        match self.cells[pos.y * self.width + pos.x] {
            Cell::Passable => true,
            Cell::Obstacle => false,
            Cell::Temporary(_) => false,
        }
    }

    pub fn neighbors(&self, pos: Vector2<usize>) -> impl Iterator<Item = Vector2<usize>> + '_ {
        let deltas = [(1isize, 0isize), (0, 1), (-1, 0), (0, -1)];
        deltas.into_iter().filter_map(move |(dx, dy)| {
            let nx = pos.x as isize + dx;
            let ny = pos.y as isize + dy;
            if nx >= 0 && ny >= 0 {
                let npos = Vector2::new(nx as usize, ny as usize);
                if self.in_bounds(npos) && self.is_passable(npos) {
                    Some(npos)
                } else {
                    None
                }
            } else {
                None
            }
        })
    }

    pub fn set_temporary(&mut self, pos: Vector2<usize>) {
        let idx = self.get_cell_location(pos);
        let cell = std::mem::replace(&mut self.cells[idx], Cell::Passable);
        self.cells[idx] = Cell::Temporary(Box::new(cell));
        self.temp_cells.insert(idx);
    }

    pub fn clear_temporary_at_idx(&mut self, idx: usize) {
        self.temp_cells.remove(&idx);
        if let Cell::Temporary(inner) = std::mem::replace(&mut self.cells[idx], Cell::Passable) {
            self.cells[idx] = *inner;
        }
    }

    pub fn clear_temporary_at(&mut self, pos: Vector2<usize>) {
        let idx = self.get_cell_location(pos);
        self.clear_temporary_at_idx(idx);
    }

    pub fn add_temporary(&mut self, pos: Vec<Vector2<usize>>) {
        for p in pos {
            self.set_temporary(p);
        }
    }

    pub fn clear_temporary(&mut self, pos: Vec<Vector2<usize>>) {
        for p in pos {
            self.clear_temporary_at(p);
        }
    }

    pub fn clear_temporary_all(&mut self) {
        for idx in self.temp_cells.clone() {
            self.clear_temporary_at_idx(idx);
        }
    }

    pub fn serialize(&self) -> pathfind::Grid2d {
        pathfind::Grid2d {
            width: self.width as i32,
            height: self.height as i32,
            grid: self.cells.iter().map(|c| c.serialize() as i32).collect(),
        }
    }
}

#[cfg(test)]
pub mod tests {
    use crate::grid::{Cell, Grid2d};
    use nalgebra::Vector2;

    #[test]
    fn test_set_and_clear_temporary_cell() {
        let map_data = vec![true; 16]; // 4x4 grid of passable cells
        let mut grid = Grid2d::new(4, 4, &map_data);
        let pos = Vector2::new(1, 1);

        grid.set_temporary(pos);
        let idx = grid.get_cell_location(pos);
        match &grid.cells[idx] {
            Cell::Temporary(inner) => assert!(matches!(**inner, Cell::Passable)),
            _ => panic!("Cell should be Temporary"),
        }
        assert!(grid.temp_cells.contains(&idx));

        grid.clear_temporary_at(pos);
        match &grid.cells[idx] {
            Cell::Passable => {}
            _ => panic!("Cell should be Passable"),
        }
        assert!(!grid.temp_cells.contains(&idx));
    }

    #[test]
    fn test_add_and_clear_multiple_temporary_cells() {
        let map_data = vec![false; 25]; // 5x5 grid of obstacle cells
        let mut grid = Grid2d::new(5, 5, &map_data);
        let positions = vec![Vector2::new(0, 0), Vector2::new(2, 2), Vector2::new(3, 4)];

        grid.add_temporary(positions.clone());
        for pos in &positions {
            let idx = grid.get_cell_location(*pos);
            assert!(matches!(&grid.cells[idx], Cell::Temporary(_)));
            assert!(grid.temp_cells.contains(&idx));
        }

        grid.clear_temporary(positions.clone());
        for pos in &positions {
            let idx = grid.get_cell_location(*pos);
            assert!(matches!(&grid.cells[idx], Cell::Obstacle));
            assert!(!grid.temp_cells.contains(&idx));
        }
    }

    #[test]
    fn test_clear_temporary_all() {
        let map_data = vec![true; 9]; // 3x3 grid of passable cells
        let mut grid = Grid2d::new(3, 3, &map_data);
        let positions = vec![Vector2::new(0, 1), Vector2::new(1, 0), Vector2::new(2, 2)];

        grid.add_temporary(positions.clone());
        assert_eq!(grid.temp_cells.len(), positions.len());

        grid.clear_temporary_all();
        assert_eq!(grid.temp_cells.len(), 0);

        for pos in &positions {
            let idx = grid.get_cell_location(*pos);
            assert!(matches!(&grid.cells[idx], Cell::Passable));
        }
    }

    #[test]
    fn test_serialize() {
        let map_data = vec![true; 4]; // 2x2 grid of passable cells
        let mut grid = Grid2d::new(2, 2, &map_data);
        grid.set_temporary(Vector2::new(0, 1));
        // Just check the size and type of the serialization
        let serialized = grid.serialize();
        assert_eq!(serialized.width, 2);
        assert_eq!(serialized.height, 2);
        assert_eq!(serialized.grid.len(), 4);
    }
}
