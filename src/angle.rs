//!
//! 用平面整数坐标点来表达的角度，用n倍4象限来表示大于360度的角度。
//!

use std::cmp::*;

use pi_null::Null;

#[derive(Debug, Clone, Copy, Default)]
pub struct Point {
    // x坐标
    pub x: isize,
    // y坐标
    pub y: isize,
}
impl Point {
    pub fn new(x: isize, y: isize) -> Self {
        Point { x, y}
    }
}
// 实现 Add trait，定义加法操作
impl std::ops::Add for Point {
    type Output = Point;
    
    fn add(self, other: Point) -> Point {
        Point::new(self.x + other.x, self.y + other.y)
    }
}

// 实现 Sub trait，定义减法操作
impl std::ops::Sub for Point {
    type Output = Point;
    
    fn sub(self, other: Point) -> Point {
        Point::new(self.x - other.x, self.y - other.y)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Angle {
    // 点
    pub p: Point,
    // 象限
    pub quadrant: isize,
}
impl Angle {
    pub fn new(p: Point) -> Self {
        let quadrant = if p.x < 0 {
            if p.y < 0 {
                2
            } else {
                1
            }
        } else if p.y < 0 {
            3
        } else {
            0
        };
        Angle { p, quadrant }
    }
    /// 向左旋转-逆时针旋转，如果旋转后的角度小于当前角度，则象限加4，代表旋转了一圈
    pub fn rotate_left(&self, p: Point) -> Self {
        let mut other = Self::new(p);
        other.quadrant += (self.quadrant >> 2) << 2;
        if &other < &self {
            other.quadrant += 4;
        }
        other
    }
    /// 向右旋转-顺时针旋转，如果旋转后的角度大于当前角度，则象限减4，代表旋转了一圈
    pub fn rotate_right(&self, p: Point) -> Self {
        let mut other = Self::new(p);
        other.quadrant += (self.quadrant >> 2) << 2;
        if &other > &self {
            other.quadrant -= 4;
        }
        other
    }
}
impl Null for Angle {
    fn null() -> Self {
        Default::default()
    }
    fn is_null(&self) -> bool {
        self.p.x == 0 && self.p.y == 0
    }
}
impl PartialEq for Angle {
    fn eq(&self, other: &Self) -> bool {
        self.quadrant == other.quadrant && self.p.y * other.p.x == self.p.x * other.p.y
    }
}
impl PartialOrd for Angle {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.quadrant.partial_cmp(&other.quadrant) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        (self.p.y * other.p.x).partial_cmp(&(self.p.x * other.p.y))
    }
}

//#![feature(test)]
#[cfg(test)]
mod test_angle {
    use crate::{angle::{Angle, Point}};

    #[test]
    fn test() {
        let a21 = Angle::new(Point::new(2, 1));
        let a22 = Angle::new(Point::new(2, 2));
        let a12 = Angle::new(Point::new(1, 2));
        let a01 = Angle::new(Point::new(0, 1));
        let a02 = Angle::new(Point::new(0, 2));
        assert_eq!(a22 > a21, true );
        assert_eq!(a12 > a22, true );
        assert_eq!(a12 > a21, true );
        assert_eq!(a01 > a12, true );
        assert_eq!(a02 > a12, true );
        assert_eq!(a02 == a01, true );
        let b12 = Angle::new(Point::new(-1, 2));
        let b22 = Angle::new(Point::new(-2, 2));
        let b21 = Angle::new(Point::new(-2, 1));
        let b42 = Angle::new(Point::new(-4, 2));
        let b10 = Angle::new(Point::new(-1, 0));
        assert_eq!(b12 > a21, true );
        assert_eq!(b21 > a01, true );
        assert_eq!(b22 > b12, true );
        assert_eq!(b21 > b22, true );
        assert_eq!(b42 == b21, true );
        assert_eq!(b10 > b42, true );
        assert_eq!(b10 > a02, true );
        let c21 = Angle::new(Point::new(-2, -1));
        let c22 = Angle::new(Point::new(-2, -2));
        let c12 = Angle::new(Point::new(-1, -2));
        let c24 = Angle::new(Point::new(-2, -4));
        let c01 = Angle::new(Point::new(0, -1));
        assert_eq!(c21 > b10, true );
        assert_eq!(c22 > c21, true );
        assert_eq!(c12 > c22, true );
        assert_eq!(c12 > c21, true );
        assert_eq!(c24 == c12, true );
        assert_eq!(c01 > c12, true );
        let d12 = Angle::new(Point::new(1, -2));
        let d22 = Angle::new(Point::new(2, -2));
        let d21 = Angle::new(Point::new(2, -1));
        let d42 = Angle::new(Point::new(4, -2));
        assert_eq!(d12 > a21, true );
        assert_eq!(d12 > b21, true );
        assert_eq!(d12 > c21, true );
        assert_eq!(d12 > c01, true );
        assert_eq!(d21 > a01, true );

        assert_eq!(d22 > d12, true );
        assert_eq!(d21 > d22, true );
        assert_eq!(d42 == d21, true );
        let e10 = d42.rotate_left(Point::new(1, 0));
        assert_eq!(e10 > d21, true );
        let e21 = d42.rotate_left(Point::new(2, 1));
        assert_eq!(e21 > e10, true );
        let f10 = a21.rotate_right(Point::new(1, -1));
        let f11 = a21.rotate_right(Point::new(-1, -1));
        assert_eq!(a21 > f10, true );
        assert_eq!(f10 > f11, true );
    }

}
