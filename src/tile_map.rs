//!
//! A*寻路的瓦片地图
//! 需指定行数和列数，每个瓦片的边长和斜角长度，默认为100和141。
//! 瓦片信息包括3个信息：瓦片是否障碍， 右边是否障碍，下边是否障碍。这些是从左到右，从上到下地进行排序的。一个矩阵表示的地图也称为瓦片地图。
//! 获得邻居时， 如果斜向对应的两个直方向的瓦片有1个不可走，则该斜方向就不可走。
//!

use crate::*;
use num_traits::Zero;
use pi_null::Null;
use pi_spatial::tilemap::*;
use std::{fmt::Debug, mem};

// 瓦片内的障碍
#[repr(C)]
#[derive(Debug, Clone)]
pub enum TileObstacle {
    Right = 1,
    Down = 2,
    Center = 4,
}
/// 获得指定位置的障碍物描述（中心是否障碍， 右边是否障碍， 下边是否障碍）
#[inline]
pub fn get_obstacle(nodes: &Vec<u8>, index: usize) -> (bool, bool, bool) {
    let i = nodes[index] as usize;
    (
        (i & TileObstacle::Center as usize) > 0,
        (i & TileObstacle::Right as usize) > 0,
        (i & TileObstacle::Down as usize) > 0,
    )
}
/// 设置flag
#[inline]
pub fn set_flag(value: usize, b: bool, flag: usize) -> usize {
    if b {
        value | flag
    } else {
        value & !flag
    }
}

pub struct TileMap {
    // 该图所有节点是否有障碍， 右边是否障碍，下边是否障碍
    pub nodes: Vec<u8>,
    // 该图最大行数
    pub row: usize,
    // 该图最大列数
    pub column: usize,
    // 该图节点的长度， 如果是整数，则必须大于10
    cell_len: usize,
    // 该图节点间斜45°的长度， 根号2 * cell_len
    oblique_len: usize,
    // 瓦片总数量
    count: usize,
}

impl TileMap {
    ///
    /// 新建一个方格图
    ///
    /// 需指定方格图的行数和列数，每个方格的边长，以及全图在场景中的起始坐标
    pub fn new(row: usize, column: usize, cell_len: usize, oblique_len: usize) -> Self {
        let len = row * column;
        let mut nodes = Vec::with_capacity(len);
        nodes.resize_with(len, Default::default);
        TileMap {
            nodes,
            row,
            column,
            cell_len,
            oblique_len,
            count: row * column,
        }
    }
    pub fn set_node_obstacle(&mut self, node: NodeIndex, tile_obstacle: u8) {
        self.nodes[node.0] = tile_obstacle;
    }
    pub fn set_node_center_obstacle(&mut self, node: NodeIndex, center_obstacle: bool) {
        self.nodes[node.0] = set_flag(
            self.nodes[node.0] as usize,
            center_obstacle,
            TileObstacle::Center as usize,
        ) as u8;
    }
    pub fn set_node_right_obstacle(&mut self, node: NodeIndex, right_obstacle: bool) {
        self.nodes[node.0] = set_flag(
            self.nodes[node.0] as usize,
            right_obstacle,
            TileObstacle::Right as usize,
        ) as u8;
    }
    pub fn set_node_down_obstacle(&mut self, node: NodeIndex, down_obstacle: bool) {
        self.nodes[node.0] = set_flag(
            self.nodes[node.0] as usize,
            down_obstacle,
            TileObstacle::Down as usize,
        ) as u8;
    }
}

impl Map<usize> for TileMap {
    type NodeIter = NodeIterator;

    fn get_neighbors(&self, cur: NodeIndex, parent: NodeIndex) -> Self::NodeIter {
        let mut arr = get_8d_neighbors(cur.0, self.column, self.count);
        let (_, r, d) = get_obstacle(&self.nodes, cur.0);
        // 检查当前点的右边和下边
        if r {
            arr[Direction::Right as usize] = Null::null();
        }
        if d {
            arr[Direction::Down as usize] = Null::null();
        }
        // 处理右边是否可达
        let i = arr[Direction::Right as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, _, down) = get_obstacle(&self.nodes, i);
                if ok {
                    arr[Direction::Right as usize] = Null::null();
                    arr[Direction::DownRight as usize] = Null::null();
                } else if down {
                    arr[Direction::DownRight as usize] = Null::null();
                }
            } else {
                arr[Direction::Right as usize] = Null::null();
                arr[Direction::DownRight as usize] = Null::null();
            }
        }
        // 处理下边是否可达
        let i = arr[Direction::Down as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, _) = get_obstacle(&self.nodes, i);
                if ok {
                    arr[Direction::Down as usize] = Null::null();
                    arr[Direction::DownRight as usize] = Null::null();
                } else if right {
                    arr[Direction::DownRight as usize] = Null::null();
                }
            } else {
                arr[Direction::Down as usize] = Null::null();
                arr[Direction::DownRight as usize] = Null::null();
            }
        }
        // 处理左边是否可达
        let i = arr[Direction::Left as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, down) = get_obstacle(&self.nodes, i);
                if ok || right {
                    arr[Direction::Left as usize] = Null::null();
                    arr[Direction::DownLeft as usize] = Null::null();
                } else if down {
                    arr[Direction::DownLeft as usize] = Null::null();
                }
            } else {
                arr[Direction::Left as usize] = Null::null();
                arr[Direction::DownLeft as usize] = Null::null();
            }
        }
        // 处理上边是否可达
        let i = arr[Direction::Up as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, down) = get_obstacle(&self.nodes, i);
                if ok || down {
                    arr[Direction::Up as usize] = Null::null();
                    arr[Direction::UpRight as usize] = Null::null();
                } else if right {
                    arr[Direction::UpRight as usize] = Null::null();
                }
            } else {
                arr[Direction::Up as usize] = Null::null();
                arr[Direction::UpRight as usize] = Null::null();
            }
        }
        // 处理左上是否可达
        let i = arr[Direction::UpLeft as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, down) = get_obstacle(&self.nodes, i);
                if ok || right || down {
                    arr[Direction::UpLeft as usize] = Null::null();
                }
            } else {
                arr[Direction::UpLeft as usize] = Null::null();
            }
        }
        // 处理右上是否可达
        let i = arr[Direction::UpRight as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, _, down) = get_obstacle(&self.nodes, i);
                if ok || down {
                    arr[Direction::UpRight as usize] = Null::null();
                }
            } else {
                arr[Direction::UpRight as usize] = Null::null();
            }
        }
        // 处理左下是否可达
        let i = arr[Direction::DownLeft as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, _) = get_obstacle(&self.nodes, i);
                if ok || right {
                    arr[Direction::DownLeft as usize] = Null::null();
                }
            } else {
                arr[Direction::DownLeft as usize] = Null::null();
            }
        }
        // 处理右下是否可达
        let i = arr[Direction::DownRight as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, _, _) = get_obstacle(&self.nodes, i);
                if ok {
                    arr[Direction::DownRight as usize] = Null::null();
                }
            } else {
                arr[Direction::DownRight as usize] = Null::null();
            }
        }

        NodeIterator { arr, index: 0 }
    }

    fn get_g(&self, cur: NodeIndex, parent: NodeIndex) -> usize {
        if cur.0 == parent.0 + 1 || cur.0 + 1 == parent.0 || cur.0 == parent.0 + self.column || cur.0 + self.column == parent.0 {
            return self.cell_len;
        }
        self.oblique_len
    }
    fn get_h(&self, cur: NodeIndex, end: NodeIndex) -> usize {
        let (from_column, from_row) = column_row(self.column, cur);
        let (to_column, to_row) = column_row(self.column, end);
        let x = (from_column - to_column).abs() as usize;
        let y = (from_row - to_row).abs() as usize;
        let (min, max) = if x <= y { (x, y) } else { (y, x) };

        let t = min;
        let r = max - min;
        // 45度斜线的长度 + 剩余直线的长度
        t * self.oblique_len + r * self.cell_len
    }
}

// 遍历邻居的迭代器
#[derive(Debug, Clone)]
pub struct NodeIterator {
    arr: [usize; 8],
    index: usize,
}

impl Iterator for NodeIterator {
    type Item = NodeIndex;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.index >= self.arr.len() {
                return None;
            }
            let i = self.arr[self.index];
            self.index += 1;
            if !i.is_null() {
                return Some(NodeIndex(i));
            }
        }
    }
}
/// 路径过滤器，合并相同方向的路径点
#[derive(Clone)]
pub struct PathFilterIter<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    result: ResultIterator<'a, N, E>,
    // 地图的列
    pub column: usize,
    pub start: NodeIndex,
    pub cur: NodeIndex,
    pub cur_row: isize,
    pub cur_column: isize,
    angle: Angle,
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> PathFilterIter<'a, N, E> {
    pub fn new(mut result: ResultIterator<'a, N, E>, column: usize) -> Self {
        let start = result.next().unwrap();
        let cur = if let Some(c) = result.next() {
            c
        } else {
            Null::null()
        };
        let (c, r) = column_row(column, start);
        let (cur_column, cur_row) = column_row(column, cur);
        let angle = Angle::new(cur_column - c, cur_row - r);
        PathFilterIter {
            result,
            column,
            start,
            cur,
            cur_row,
            cur_column,
            angle,
        }
    }
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> Iterator
    for PathFilterIter<'a, N, E>
{
    type Item = NodeIndex;
    fn next(&mut self) -> Option<Self::Item> {
        if self.start.is_null() {
            return None;
        }
        if self.cur.is_null() {
            return Some(mem::replace(&mut self.start, Null::null()));
        }
        for node in &mut self.result {
            let (c, r) = column_row(self.column, node);
            let angle = Angle::new(c - self.cur_column, r - self.cur_row);
            self.cur_row = r;
            self.cur_column = c;
            if angle != self.angle {
                self.angle = angle;
                let node = mem::replace(&mut self.cur, node);
                return Some(mem::replace(&mut self.start, node));
            }
            self.cur = node;
        }
        let node = mem::replace(&mut self.cur, Null::null());
        Some(mem::replace(&mut self.start, node))
    }
}

/// 路径平滑器，合并直接可达的路径点，采用佛洛依德路径平滑算法（FLOYD），
/// 判断在地图上两个点间的每个格子是否都可以走，算出直线划过的格子，其实就是画直线的算法，
/// https://zhuanlan.zhihu.com/p/34074528
#[derive(Clone)]
pub struct PathSmoothIter<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    result: PathFilterIter<'a, N, E>,
    map: &'a TileMap,
    start: NodeIndex,
    start_cr: (isize, isize),
    cur: NodeIndex,
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> PathSmoothIter<'a, N, E> {
    pub fn new(mut result: PathFilterIter<'a, N, E>, map: &'a TileMap) -> Self {
        let start = result.next().unwrap();
        let start_cr = column_row(map.column, start);

        let cur = if let Some(c) = result.next() {
            c
        } else {
            Null::null()
        };
        PathSmoothIter { result, map, start, start_cr, cur }
    }
    /// 判断是否地图上直线可达
    fn test(map: & TileMap, start_cr: (isize, isize), node: NodeIndex) -> bool {
        let cr = column_row(map.column, node);
        let it = bresenham(start_cr, cr);
        let stop = it.0.coord.1;
        let mut last = 0;
        if it.1 {
            for (y, x) in it.0 {
                if x == stop { // 如果和终点直线连通，则可判断为直线可达
                    return true
                }
            }
        }else{
            for (x, y) in it.0 {
                if y == stop {// 如果和终点直线连通，则可判断为直线可达
                    return true
                }
            }
        }
        true
    }
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> Iterator
    for PathSmoothIter<'a, N, E>
{
    type Item = NodeIndex;
    fn next(&mut self) -> Option<Self::Item> {
        if self.start.is_null() {
            return None;
        }
        if self.cur.is_null() {
            return Some(mem::replace(&mut self.start, Null::null()));
        }
        for node in &mut self.result {
            // 判断该点和起点是否直线可达
            let ok = Self::test(self.map, self.start_cr, node);
            if !ok {
                let node = mem::replace(&mut self.cur, node);
                return Some(mem::replace(&mut self.start, node));
            }
            self.cur = node;
        }
        let node = mem::replace(&mut self.cur, Null::null());
        Some(mem::replace(&mut self.start, node))
    }
}

/// 获得指定位置瓦片的列行
pub fn column_row(column: usize, index: NodeIndex) -> (isize, isize) {
    ((index.0 % column) as isize, (index.0 / column) as isize)
}

//#![feature(test)]
#[cfg(test)]
mod test_tilemap {
    use crate::*;
    use rand_core::SeedableRng;
    use test::Bencher;
    #[test]
    fn test2() {
        let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(11, 11, 100, 141);
        map.nodes[48] = 4;
        map.nodes[49] = 4;
        map.nodes[50] = 4;
        map.nodes[59] = 4;
        map.nodes[60] = 4;
        map.nodes[61] = 4;
        map.nodes[70] = 4;
        map.nodes[71] = 4;
        map.nodes[72] = 4;

        let column = map.column;

        let mut astar: AStar<usize, Entry<usize>> = AStar::with_capacity(map.column * map.row, 100);


        let start = NodeIndex(120);
        let end = NodeIndex(0);

        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        println!("r: {:?}", r);

        let mut c = 0;
        for r in PathFilterIter::new(astar.result_iter(end), map.column) {
            println!("x:{},y:{}", r.0 % column, r.0 / column);
            c += 1;
        }
        println!("c:{}", c);

        let start = NodeIndex(0);
        let end = NodeIndex(120);

        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        println!("r: {:?}", r);

        let mut c = 0;
        for r in PathFilterIter::new(astar.result_iter(end), map.column) {
            println!("x:{},y:{}", r.0 % column, r.0 / column);
            c += 1;
        }
        println!("c:{}", c);

    }
    #[test]
    fn test3() {
        let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(1000, 1000, 100, 141);
        map.nodes[88 + 88 * map.column] = TileObstacle::Center as u8;
        map.nodes[65 + 64 * map.column] = TileObstacle::Center as u8;
        map.nodes[54 + 55 * map.column] = TileObstacle::Center as u8;
        map.nodes[44 + 44 * map.column] = TileObstacle::Center as u8;
        map.nodes[33 + 33 * map.column] = TileObstacle::Center as u8;
        let column = map.column;
        let x1 = 999; //rng.next_u32() as usize%map.column;
        let y1 = 999; //rng.next_u32()as usize %map.row;
        let x2 = 1; //rng.next_u32() as usize%map.column;
        let y2 = 1; //rng.next_u32()as usize %map.row;
        println!("x1:{},y1:{}, x2:{},y2:{}", x1, y1, x2, y2);
        let mut astar: AStar<usize, Entry<usize>> = AStar::with_capacity(map.column * map.row, 100);
        let start = NodeIndex(x1 + y1 * map.column);
        let end = NodeIndex(x2 + y2 * map.column);
        let r = astar.find(start, end, 30000, &mut map, make_neighbors);

        println!("r: {:?}", r);
        let mut c = 0;
        for r in PathFilterIter::new(astar.result_iter(end), map.column) {
            println!("x:{},y:{}", r.0 % column, r.0 / column);
            c += 1;
        }
        println!("c:{}", c);
    }

    #[bench]
    fn bench_test1(b: &mut Bencher) {
        let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(100, 100, 100, 141);
        map.nodes[88 + 88 * map.column] = 4;
        map.nodes[65 + 64 * map.column] = 4;
        map.nodes[54 + 55 * map.column] = 4;
        map.nodes[44 + 44 * map.column] = 4;
        map.nodes[33 + 33 * map.column] = 4;

        let x1 = 99; //rng.next_u32() as usize%map.column;
        let y1 = 99; //rng.next_u32()as usize %map.row;
        let x2 = 1; //rng.next_u32() as usize%map.column;
        let y2 = 1; //rng.next_u32()as usize %map.row;
                    //println!("x1:{},y1:{}, x2:{},y2:{}", x1, y1, x2, y2);
        let mut astar: AStar<usize, Entry<usize>> = AStar::with_capacity(map.column * map.row, 100);
        b.iter(move || {
            let start = NodeIndex(x1 + y1 * map.column);
            let end = NodeIndex(x2 + y2 * map.column);
            let r = astar.find(start, end, 30000, &mut map, make_neighbors);
            let mut vec: Vec<NodeIndex> = Vec::with_capacity(100);
            for r in PathFilterIter::new(astar.result_iter(end), map.column) {
                vec.push(r);
                // println!("x:{},y:{}", r.0%map.column, r.0/map.column);
            }
        });
    }
}
