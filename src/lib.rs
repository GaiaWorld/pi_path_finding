//! 寻路 模块
//!
//! finder A* 主体
//!
//! tile_map 模块，实现了方格数据的A*寻路；
//!
//! nav_mesh 模块，实现了 3维导航网格的A*寻路（包含路径拉直）
//!

#![feature(test)]
extern crate test;


pub mod finder;
pub mod normal;
pub mod tile_map;
pub mod nav_mesh;
pub mod angle;
pub mod bresenham;
pub mod web;