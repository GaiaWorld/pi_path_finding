//! 寻路 模块
//!
//! astar 函数：用抽象的Node的概念，实现了通用的A*逻辑
//!
//! tile 模块，实现了方格数据的A*寻路；
//!
//! nav_mesh 模块，实现了 3维导航网格的A*寻路（包含路径拉直）
//!

#![feature(test)]
extern crate test;

mod finder;
mod normal;
mod tile_map;
mod jump_point;

pub use finder::*;
pub use normal::*;
pub use jump_point::*;
pub use tile_map::*;

// mod fast_astar;
// // mod astar;
// mod jps;
// mod nav_mesh;
// mod tile;
// mod test_jps;
// mod test_jps_astar;
// mod jps_plus;

// // pub use astar::*;
// pub use jps::*;
// pub use nav_mesh::*;
// pub use fast_astar::*;
// pub use tile::*;
// pub use test_jps::*;
// pub use test_jps_astar::*;
// pub use jps_plus::*;