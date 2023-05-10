//!
//! Bresenham直线算法
//!
use std::mem::*;

use crate::base::Point;

// use crate::Point;


#[derive(Debug, Clone, Default)]
pub struct Bresenham {
    // 开始坐标
    pub start: Point,
    // 结束坐标
    pub end: Point,
    // x的单位 -1 或 1
    pub step: isize,
    // 斜率的(dy, dx, dx*y1-dy*x1)
    pub steep: (isize, f32, isize),
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
        let steep = if end.x == start.x || end.y == start.y {
            (0, 1.0, start.y)
        } else {
            let dy = end.y - start.y;
            let dx = end.x - start.x;
            // dx*y1-dy*x1
            (dy, dx as f32, dx*start.y - dy*start.x)
        };
        let step = if end.x > start.x { 1 } else { -1 };
        Bresenham {
            start,
            end,
            step,
            steep,
            xy_change,
        }
    }
    pub fn is_over(&self) -> bool {
        self.start.x == self.end.x
    }
    pub fn y(&self, x: isize) -> isize {
        // y = (dy*x + dx*y1-dy*x1) / dx
        ((x * self.steep.0 + self.steep.2) as f32 / self.steep.1).round() as isize
    }
    pub fn step(&mut self) {
        self.start.x += self.step;        
        self.start.y = self.y(self.start.x);
    }
}

