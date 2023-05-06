//!
//! Mipmap分级地图
//!

use bitvec::prelude::*;
use pi_null::Null;
use std::mem::{transmute, MaybeUninit};

use crate::{Aabb, NodeIndex, Point};

// 四方向枚举
#[repr(C)]
#[derive(Debug, Clone)]
pub enum Location {
    UpLeft = 0,
    UpRight = 1,
    DownLeft = 2,
    DownRight = 3,
}

/// 获得指定位置瓦片及方向的包含自身及周围的四个瓦片坐标，第一个坐标一定是min_xy，最后一个坐标是max_xy，顺序遵循0(左上) 1(右上) 2(左下) 3(右下)
pub fn get_round(p: Point, d: Location, width: isize, height: isize) -> ([Point; 4], usize) {
    // 创建一个长度为 4 的未初始化的数组
    let array: [MaybeUninit<Point>; 4] = unsafe { MaybeUninit::uninit().assume_init() };
    let mut arr: [Point; 4] = unsafe { transmute::<[MaybeUninit<Point>; 4], [Point; 4]>(array) };
    let mut i = 0;
    fn push(arr: &mut [Point], i: &mut usize, p: Point) {
        arr[*i] = p;
        *i += 1;
    }
    match d {
        Location::UpLeft => {
            if p.x > 0 {
                if p.y > 0 {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y - 1)); // 0
                    push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 1
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 2
                } else {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 2
                }
            } else if p.y > 0 {
                push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 1
            }
            push(&mut arr, &mut i, p); // 3
        }
        Location::UpRight => {
            if p.x + 1 < width {
                if p.y > 0 {
                    push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 0
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y - 1)); // 1
                    push(&mut arr, &mut i, p); // 2
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 3
                } else {
                    push(&mut arr, &mut i, p); // 2
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 3
                }
            } else if p.y > 0 {
                push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 0
                push(&mut arr, &mut i, p); // 2
            } else {
                push(&mut arr, &mut i, p); // 2
            }
        }
        Location::DownLeft => {
            if p.x > 0 {
                if p.y + 1 < height {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 0
                    push(&mut arr, &mut i, p); // 1
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y + 1)); // 2
                    push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 3
                } else {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 0
                    push(&mut arr, &mut i, p); // 1
                }
            } else if p.y + 1 < height {
                push(&mut arr, &mut i, p); // 1
                push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 3
            } else {
                push(&mut arr, &mut i, p); // 1
            }
        }
        Location::DownRight => {
            push(&mut arr, &mut i, p); // 0
            if p.x + 1 < width {
                if p.y + 1 < height {
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 1
                    push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 2
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y + 1)); // 3
                } else {
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 1
                }
            } else if p.y + 1 < height {
                push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 2
            }
        }
    }
    return (arr, i);
}
const RESOLUTIONS: [usize; 5] = [2, 3, 4, 6, 8];
const RESOLUTIONS_2: [usize; 5] = [4, 9, 16, 36, 64];
#[derive(Debug, Clone, Default)]
pub struct MipMap {
    // 所有地图的节点
    pub nodes: Vec<u8>,
    // 该图宽度
    pub width: usize,
    // 该图高度
    pub height: usize,
    // 0级地图的节点总数量
    amount: usize,
    // 分级地图
    maps: [Map; 5],
}
impl MipMap {
    pub fn new(width: usize, height: usize) -> Self {
        let amount = width * height;
        // 0级用bit位存储，为长度除8，1-5级用byte字节存储，为长度除1/4+1/9+1/16+1/36+1/64,
        let len = amount.div_ceil(8)
            + amount.div_ceil(RESOLUTIONS_2[0])
            + amount.div_ceil(RESOLUTIONS_2[1])
            + amount.div_ceil(RESOLUTIONS_2[2])
            + amount.div_ceil(RESOLUTIONS_2[3])
            + amount.div_ceil(RESOLUTIONS_2[4]);
        let mut nodes = Vec::with_capacity(len);
        nodes.resize_with(len, Default::default);
        let mut maps = [Map::default(); 5];
        let mut offset = amount.div_ceil(8);
        for i in 0..RESOLUTIONS.len() {
            let r = RESOLUTIONS[i];
            let w = width.div_ceil(r);
            let h = height.div_ceil(r);
            maps[i].offset = offset;
            maps[i].resolution = r;
            maps[i].limit = r * r;
            maps[i].width = w;
            maps[i].height = h;
            offset += w * h;
        }
        MipMap {
            nodes,
            width,
            height,
            amount,
            maps,
        }
    }
    // 判断一个点是否被设置
    #[inline]
    pub fn is_true(&self, node: NodeIndex) -> bool {
        if node.0 >= self.amount {
            return false;
        }
        self.is(node.0)
    }
    // 设置一个点为ok
    #[inline]
    pub fn set_true(&mut self, node: NodeIndex) -> bool {
        if node.0 >= self.amount {
            return false;
        }
        let bits = self.nodes.view_bits_mut::<Lsb0>();
        if bits[node.0] {
            return false;
        }
        bits.set(node.0, true);
        let x = node.0 % self.width; // get the x and y position of the node in the array.  (0,0) is the top left.
        let y = node.0 / self.width;
        for m in &mut self.maps {
            m.set_true(&mut self.nodes, x, y) // add one to the cell value
        }
        true
    }
    // 设置一个点为false
    #[inline]
    pub fn set_false(&mut self, node: NodeIndex) -> bool {
        if node.0 >= self.amount {
            return false;
        }
        let bits = self.nodes.view_bits_mut::<Lsb0>();
        if !bits[node.0] {
            return false;
        }
        bits.set(node.0, false);
        let x = node.0 % self.width; // get the x and y position of the node in the array.  (0,0) is the top left.
        let y = node.0 / self.width;
        for m in &mut self.maps {
            m.set_false(&mut self.nodes, x, y) // add one to the cell value
        }
        true
    }
    // 将源点设为false，将目标点设为true，要求源点的值为true，目标点的值为false
    #[inline]
    pub fn move_to(&mut self, src: NodeIndex, dest: NodeIndex) -> bool {
        if src.0 >= self.amount {
            return false;
        }
        if dest.0 >= self.amount {
            return false;
        }
        let bits = self.nodes.view_bits_mut::<Lsb0>();
        if bits[src.0] {
            return false;
        }
        if !bits[dest.0] {
            return false;
        }
        bits.set(src.0, false);
        bits.set(dest.0, true);
        true
    }
    #[inline]
    fn is(&self, id: usize) -> bool {
        let bits = self.nodes.view_bits::<Lsb0>();
        bits[id]
    }
    // 计算一个点周围可以容纳下的范围，如果当前点不可用，则返回0
    pub fn find_round(&self, node: NodeIndex, d: Location, count: usize) -> (Aabb, usize) {
        if self.is_true(node) {
            let p = Point::new(Null::null(), Null::null());
            return (Aabb::new(p, p), 0);
        }
        let p = Point::new((node.0 % self.width) as isize, (node.0 / self.width) as isize);
        if count <= 1 {
            // 如果只查找1个点，则可以返回当前点
            return (Aabb::new(p, p.add(1)), 1);
        }
        if count <= 4 {
            // 如果查找少于等于4个点，则尝试在0级地图上寻找
            let (arr, len) = get_round(p, d, self.width as isize, self.height as isize);
            if count <= len {
                // 遍历点，判断是否有设置
                let mut c = 0;
                for i in 0..len {
                    let id = arr[i].x as usize + arr[i].y as usize * self.width; // index in nodes array, not a node index.
                    if !self.is(id) {
                        c += 1;
                    }
                }
                if count <= c {
                    return (Aabb::new(arr[0], arr[len - 1].add(1)), c);
                }
            }
        }
        // 在分级地图上查找
        for m in self.maps {
            if m.limit * 4 < count {
                continue;
            }
            if m.width == 1 || m.height == 1 {
                break;
            }
            let mut r = m.find_round(&self.nodes, p, count);
            if r.1 == 0 {
                continue;
            }
            r.0.min.mul(m.resolution as isize);
            r.0.max.mul(m.resolution as isize);
            if r.0.max.x > self.width as isize {
                r.0.max.x = self.width as isize; // truncate it to the width of the map. this is a bit of a hack.
            }
            if r.0.max.y > self.height as isize {
                r.0.max.y = self.height as isize; // truncate it to the width of the map. this is a bit of a hack.
            }
            return r;
        }
        (Aabb::new(p, p), 0)
    }
    // 获得指定范围所有可用的点
    pub fn get_list(&self, aabb: Aabb, result: &mut Vec<NodeIndex>) {
        let bits = self.nodes.view_bits::<Lsb0>();
        for i in aabb.min.y..aabb.max.y {
            let iw = i as usize * self.width;
            for j in aabb.min.x..aabb.max.x {
                let id = j as usize + iw;
                if !bits[id] {
                    result.push(NodeIndex(id));
                }
            }
        }
    }
}
// 地图
#[derive(Debug, Clone, Copy, Default)]
struct Map {
    // 该分级地图所在的偏移位置
    pub offset: usize,
    // 宽度
    pub width: usize,
    // 高度
    pub height: usize,
    // 分辨率
    resolution: usize,
    // 每个单元的最大值，分辨率的平方
    limit: usize,
}
impl Map {
    // 设置一个点为false
    #[inline]
    pub fn set_true(&mut self, nodes: &mut Vec<u8>, x: usize, y: usize) {
         nodes[self.offset + (x/self.resolution) + (y/self.resolution) * self.width] += 1;
    }
    // 设置一个点为false
    #[inline]
    pub fn set_false(&mut self, nodes: &mut Vec<u8>, x: usize, y: usize) {
        nodes[self.offset + (x/self.resolution) + (y/self.resolution) * self.width] -= 1;
    }
    // 计算一个点周围可以容纳下的范围
    pub fn find_round(&self, nodes: &Vec<u8>, point: Point, count: usize) -> (Aabb, usize) {
        let p = point.div(self.resolution as isize);
        let pp = p.mul(self.resolution as isize);
        let mut d = 0u32;
        if point.x + point.x >= pp.x + pp.x + self.resolution as isize {
            d |= 1;
        }
        if point.y + point.y >= pp.y + pp.y + self.resolution as isize {
            d |= 2;
        }
        let (arr, len) = get_round(
            p,
            unsafe { transmute(d) },
            self.width as isize,
            self.height as isize,
        );
        if count <= len * self.limit {
            // 遍历点，获得可用数量
            let mut c = 0;
            for i in 0..len {
                c += self.get(nodes, arr[i]);
            }
            if count <= c {
                return (Aabb::new(arr[0], arr[len - 1].add(1)), c);
            }
        }
        (Aabb::new(p, p), 0)
    }
    // 返回指定点的所在位置的可用数量
    pub fn get(&self, nodes: &Vec<u8>, p: Point) -> usize {
        return self.limit
            - (nodes[self.offset + p.x as usize + p.y as usize * self.width] as usize);
    }
}


//#![feature(test)]
#[cfg(test)]
mod test_mipmap {
    use crate::*;
    //use rand_core::SeedableRng;
    use test::Bencher;
    #[test]
    fn test2() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = MipMap::new(11, 11);
        map.set_true(NodeIndex(48));
        map.set_true(NodeIndex(49));
        map.set_true(NodeIndex(50));
        map.set_true(NodeIndex(59));
        map.set_true(NodeIndex(60));
        map.set_true(NodeIndex(61));
        map.set_true(NodeIndex(70));
        map.set_true(NodeIndex(71));
        map.set_true(NodeIndex(72));


        println!("c:{}", 1);

    }
}