//!
//! Bresenham直线算法
//!
use std::mem::*;


#[derive(Debug, Clone, Default)]
pub struct Bresenham {
    // 开始坐标
    pub start: (isize, isize),
    // 结束坐标
    pub end: (isize, isize),
    // 斜率
    pub steep: (isize, f32),
    // 小步数
    pub float: f32,
    // 是否坐标交换（斜率在45-135度之间）
    pub xy_change: bool,
}
impl Bresenham {
    pub fn new(mut start: (isize, isize), mut end: (isize, isize)) -> Self {
        let xy_change = if (end.0 - start.0).abs() >= (end.1 - start.1).abs() {
            false
        }else {
            swap(&mut start.0, &mut start.1);
            swap(&mut end.0, &mut end.1);
            true
        };
        let f = if end.0 == start.0 {
            0.0f32
        } else if end.1 > start.1 {
            (end.1 - start.1) as f32 / (end.0 - start.0).abs() as f32 + f32::EPSILON
        } else if end.1 < start.1 {
            (end.1 - start.1) as f32 / (end.0 - start.0).abs() as f32 - f32::EPSILON
        }else{
            0.0f32
        };
        let steep = (if end.0 > start.0 { 1 } else { -1 }, f);
        Bresenham {
            start,
            end,
            steep,
            float: start.1 as f32,
            xy_change,
        }
    }
    pub fn is_over(&self) -> bool {
        self.start.0 == self.end.0
    }
    pub fn step(&mut self) {
        self.start.0 += self.steep.0;
        self.float += self.steep.1;
        self.start.1 = self.float as isize;

    }
}

