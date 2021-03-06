//!
//! A*寻路的网格图，需指定方格图的行数和列数，每个方格的边长，边长如果是整数，则必须大于10
//!

use crate::*;
use nalgebra::Scalar;
use num_traits::{cast::AsPrimitive, FromPrimitive, Num};

/// ## 方格图
/// 需指定方格图的行数和列数，每个方格的边长，边长如果是整数，则必须大于10
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
///
pub struct NormalTileMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    // 该图所有节点
    pub nodes: Vec<bool>,
    // 该图最大行数
    pub row: usize,
    // 该图最大列数
    pub column: usize,
    // 该图节点的长度， 如果是整数，则必须大于10
    cell_len: N,
    // 该图节点间斜45°的长度， 根号2 * cell_len
    oblique_len: N,
}

impl<N> NormalTileMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    ///
    /// 新建一个方格图
    ///
    /// 需指定方格图的行数和列数，每个方格的边长，以及全图在场景中的起始坐标
    pub fn new(row: usize, column: usize, cell_len: N) -> Self {
        let len = row * column;
        let mut nodes = Vec::with_capacity(len);
        nodes.resize_with(len, Default::default);
        NormalTileMap {
            nodes,
            row,
            column,
            cell_len,
            oblique_len: FromPrimitive::from_f32(cell_len.as_() as f32 * std::f32::consts::SQRT_2)
                .unwrap(),
        }
    }
}

impl<N> Map<N> for NormalTileMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    type NodeIter = TileNodeIterator;

    fn get_neighbors<'a>(&'a self, cur: NodeIndex, parent: NodeIndex) -> Self::NodeIter {
        let mut iter = TileNodeIterator {
            arr: [0; 8],
            index: 0,
        };
        let row = cur.0 / self.column;
        let column = cur.0 % self.column;
        if row > 0 {
            iter.add(row - 1, column, self.column, parent.0, &self.nodes);
            if row + 1 < self.row {
                iter.add(row + 1, column, self.column, parent.0, &self.nodes);
                if column > 0 {
                    iter.add(row, column - 1, self.column, parent.0, &self.nodes);
                    iter.add(row - 1, column - 1, self.column, parent.0, &self.nodes);
                    iter.add(row + 1, column - 1, self.column, parent.0, &self.nodes);
                    if column + 1 < self.column {
                        iter.add(row, column + 1, self.column, parent.0, &self.nodes);
                        iter.add(row - 1, column + 1, self.column, parent.0, &self.nodes);
                        iter.add(row + 1, column + 1, self.column, parent.0, &self.nodes);
                    }
                } else if column + 1 < self.column {
                    iter.add(row, column + 1, self.column, parent.0, &self.nodes);
                    iter.add(row - 1, column + 1, self.column, parent.0, &self.nodes);
                    iter.add(row + 1, column + 1, self.column, parent.0, &self.nodes);
                }
            } else {
                if column > 0 {
                    iter.add(row, column - 1, self.column, parent.0, &self.nodes);
                    iter.add(row - 1, column - 1, self.column, parent.0, &self.nodes);
                    if column + 1 < self.column {
                        iter.add(row, column + 1, self.column, parent.0, &self.nodes);
                        iter.add(row - 1, column + 1, self.column, parent.0, &self.nodes);
                    }
                } else if column + 1 < self.column {
                    iter.add(row, column + 1, self.column, parent.0, &self.nodes);
                    iter.add(row - 1, column + 1, self.column, parent.0, &self.nodes);
                }
            }
        } else if row + 1 < self.row {
            iter.add(row + 1, column, self.column, parent.0, &self.nodes);
            if column > 0 {
                iter.add(row, column - 1, self.column, parent.0, &self.nodes);
                iter.add(row + 1, column - 1, self.column, parent.0, &self.nodes);
                if column + 1 < self.column {
                    iter.add(row, column + 1, self.column, parent.0, &self.nodes);
                    iter.add(row + 1, column + 1, self.column, parent.0, &self.nodes);
                }
            } else if column + 1 < self.column {
                iter.add(row, column + 1, self.column, parent.0, &self.nodes);
                iter.add(row + 1, column + 1, self.column, parent.0, &self.nodes);
            }
        }
        iter
    }

    fn get_g(&self, cur: NodeIndex, parent: NodeIndex) -> N {
        let to_row = cur.0 / self.column;
        let from_row = parent.0 / self.column;
        if from_row == to_row {
            return self.cell_len;
        }
        let to_column = cur.0 % self.column;
        let from_column = parent.0 % self.column;
        if from_column == to_column {
            return self.cell_len;
        }
        self.oblique_len
    }
    fn get_h(&self, cur: NodeIndex, end: NodeIndex) -> N {
        let from_row = cur.0 / self.column;
        let to_row = end.0 / self.column;
        let from_column = cur.0 % self.column;
        let to_column = end.0 % self.column;
        let x = (from_column as isize - to_column as isize).abs() as usize;
        let y = (from_row as isize - to_row as isize).abs() as usize;
        let (min, max) = if x <= y { (x, y) } else { (y, x) };

        let t: N = FromPrimitive::from_usize(min).unwrap();
        let r: N = FromPrimitive::from_usize(max - min).unwrap();
        // 45度斜线的长度 + 剩余直线的长度
        t * self.oblique_len + r * self.cell_len
    }
}

impl<N> TileMap for NormalTileMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    /// 判断该点是否可以行走
    fn is_ok(&self, cur: NodeIndex) -> bool {
        self.nodes[cur.0]
    }
    /// 获得地图的行
    fn get_row(&self) -> usize {
        self.row
    }
    /// 获得地图的列
    fn get_column(&self) -> usize {
        self.column
    }
}

// 遍历邻居的迭代器
pub struct TileNodeIterator {
    arr: [usize; 8],
    index: usize,
}
impl TileNodeIterator {
    fn add(
        &mut self,
        row: usize,
        column: usize,
        map_column: usize,
        parent: usize,
        vec: &Vec<bool>,
    ) {
        let i = row * map_column + column;
        if i != parent && !vec[i] {
            self.arr[self.index] = i;
            self.index += 1;
        }
    }
}
impl Iterator for TileNodeIterator {
    type Item = NodeIndex;
    fn next(&mut self) -> Option<Self::Item> {
        if self.index == 0 {
            return None;
        }
        self.index -= 1;
        Some(NodeIndex(self.arr[self.index]))
    }
}

//#![feature(test)]
#[cfg(test)]
mod test_tilemap {
    extern crate pcg_rand;
    extern crate rand_core;

    use crate::*;
    use rand_core::{RngCore, SeedableRng};
    use test::Bencher;

    #[test]
    fn test4() {
        let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = NormalTileMap::new(1000, 1000, 100usize);
        map.nodes[88 + 88 * map.column] = true;
        map.nodes[65 + 64 * map.column] = true;
        map.nodes[54 + 55 * map.column] = true;
        map.nodes[44 + 44 * map.column] = true;
        map.nodes[33 + 33 * map.column] = true;
        let column = map.column;
        let x1 = 999; //rng.next_u32() as usize%map.column;
        let y1 = 999; //rng.next_u32()as usize %map.row;
        let x2 = 1; //rng.next_u32() as usize%map.column;
        let y2 = 1; //rng.next_u32()as usize %map.row;
        println!("x1:{},y1:{}, x2:{},y2:{}", x1, y1, x2, y2);
        let mut astar: AStar<usize, Entry<usize>> = AStar::with_capacity(map.column * map.row, 100);
        let start = NodeIndex(x1 + y1 * map.column);
        let end = NodeIndex(x2 + y2 * map.column);
        let r = astar.find(start, end, 30000, &mut |cur, end, finder| {
            make_neighbors(&mut map, cur, end, finder)
        });

        println!("r: {:?}", r);
        let mut c = 0;
        for r in astar.result_iter(end) {
            println!("x:{},y:{}", r.0 % column, r.0 / column);
            c += 1;
        }
        println!("c:{}", c);
    }

    #[bench]
    fn bench_test(b: &mut Bencher) {
        let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = NormalTileMap::new(100, 100, 100usize);
        map.nodes[88 + 88 * map.column] = true;
        map.nodes[65 + 64 * map.column] = true;
        map.nodes[54 + 55 * map.column] = true;
        map.nodes[44 + 44 * map.column] = true;
        map.nodes[33 + 33 * map.column] = true;

        let x1 = 99; //rng.next_u32() as usize%map.column;
        let y1 = 99; //rng.next_u32()as usize %map.row;
        let x2 = 1; //rng.next_u32() as usize%map.column;
        let y2 = 1; //rng.next_u32()as usize %map.row;
                    //println!("x1:{},y1:{}, x2:{},y2:{}", x1, y1, x2, y2);
        let mut astar: AStar<usize, Entry<usize>> = AStar::with_capacity(map.column * map.row, 100);
        b.iter(move || {
            let start = NodeIndex(x1 + y1 * map.column);
            let end = NodeIndex(x2 + y2 * map.column);
            let r = astar.find(start, end, 30000, &mut |cur, end, finder| {
                make_neighbors(&mut map, cur, end, finder)
            });
            let mut vec: Vec<NodeIndex> = Vec::with_capacity(100);
            for r in astar.result_iter(end) {
                vec.push(r);
                // println!("x:{},y:{}", r.0%map.column, r.0/map.column);
            }
        });
    }
}
