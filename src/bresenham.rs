//!
//! Bresenham直线算法
//!
use std::mem::*;

use crate::Point;


#[derive(Debug, Clone, Default)]
pub struct Bresenham {
    // 开始坐标
    pub start: Point,
    // 结束坐标
    pub end: Point,
    // 斜率
    pub steep: (isize, f32),
    // 小步数
    pub float: f32,
    // 是否坐标交换（斜率在45-135度之间）
    pub xy_change: bool,
}
impl Bresenham {
    pub fn new(mut start: Point, mut end: Point) -> Self {
        let xy_change = if (end.x - start.x).abs() >= (end.y - start.y).abs() {
            false
        }else {
            swap(&mut start.x, &mut start.y);
            swap(&mut end.x, &mut end.y);
            true
        };
        let f = if end.x == start.x {
            0.0f32
        } else if end.y > start.y {
            (end.y - start.y) as f32 / (end.x - start.x).abs() as f32 + f32::EPSILON
        } else if end.y < start.y {
            (end.y - start.y) as f32 / (end.x - start.x).abs() as f32 - f32::EPSILON
        }else{
            0.0f32
        };
        let steep = (if end.x > start.x { 1 } else { -1 }, f);
        Bresenham {
            start,
            end,
            steep,
            float: start.y as f32,
            xy_change,
        }
    }
    pub fn is_over(&self) -> bool {
        self.start.x == self.end.x
    }
    pub fn step(&mut self) {
        self.start.x += self.steep.0;
        self.float += self.steep.1;
        self.start.y = self.float as isize;
    }
}

