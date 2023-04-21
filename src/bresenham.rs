//!
//! Bresenham直线算法
//!

pub fn bresenham(start: (isize, isize), end: (isize, isize)) -> (Iter, bool) {
    let x = end.0 - start.0;
    let y = end.1 - start.1;
    if x.abs() >= y.abs() {
        (Iter::new((x, y)), false)
    }else {
        (Iter::new((y, x)), true)
    }
}
#[derive(Debug, Clone, Default)]
pub struct Iter {
    // xy坐标
    pub coord: (isize, isize),
    // 斜率
    pub steep: (isize, f32),
    // 步数
    pub cur: (isize, f32),
}
impl Iter {
    fn new(coord: (isize, isize)) -> Self {
        let e = if coord.1 >= 0 {
            f32::EPSILON
        } else {
            -f32::EPSILON
        };
        let f = coord.1 as f32 / coord.0.abs() as f32 + e;
        let steep = (if coord.0 >= 0 { 1 } else { -1 }, f);
        Iter {
            coord,
            steep,
            cur: (0, 0.0f32),
        }
    }
}
impl Iterator for Iter {
    type Item = (isize, isize);
    fn next(&mut self) -> Option<Self::Item> {
        if self.cur.0 == self.coord.0 {
            return None;
        }
        self.cur.0 += self.steep.0;
        self.cur.1 += self.steep.1;
        Some((self.cur.0, self.cur.1 as isize))
    }
}
