//!
//! 跳点版的A*算法需要的节点条目和创建节点邻居的函数
//!

use std::cmp::PartialOrd;

use num_traits::Zero;
use bitvec::prelude::*;

use crate::*;

/// ## A*寻路的抽象地图
///
/// 需要实现获取遍历邻居节点的迭代器的方法， 其中迭代器`Item`指定为`Node`，表示节点在全图节点集合的索引
///
pub trait TileMap {

    /// 判断该点是否可以行走
    fn is_ok(&self, cur: NodeIndex) -> bool;
    /// 获得地图的行
    fn get_row(&self) -> usize;
    /// 获得地图的列
    fn get_column(&self) -> usize;
}

// 八方向枚举
pub enum Direction {
    Up = 1,
    Down = 2,
    Left = 4,
    Right = 8,
    UpRight = 16,
    DownLeft = 32,
    UpLeft = 64,
    DownRight = 128,
}
impl Direction {
    /// 获得方向对应的节点调整值
    fn get_fix<M: TileMap>(&self, map: &M) -> isize {
        match self {
            Direction::Up => -(map.get_column() as isize),
            Direction::Down => map.get_column() as isize,
            Direction::Left => -1,
            Direction::Right => 1,
            Direction::UpRight => -(map.get_column() as isize) + 1,
            Direction::DownLeft => map.get_column() as isize - 1,
            Direction::UpLeft => -(map.get_column() as isize) - 1,
            Direction::DownRight => map.get_column() as isize + 1,
        }
    }
}


/// 跳点节点条目
pub struct JPEntry<N: PartialOrd + Zero + Copy> {
    direction: u8,
    entry: Entry<N>,
}
impl<N: PartialOrd + Zero + Copy> Default for JPEntry<N> {
    fn default() -> Self {
        JPEntry {
            direction: 0,
            entry: Entry::default(),
        }
    }
}
impl<N: PartialOrd + Zero + Copy> NodeEntry<N> for JPEntry<N> {
    fn g(&self) -> N {
        self.entry.g
    }
    fn h(&self) -> N {
        self.entry.h
    }
    fn state(&self) -> NodeState {
        self.entry.state()
    }
    fn parent(&self) -> NodeIndex {
        self.entry.parent
    }
}

/// 创建节点邻居的函数
/// 直达函数： 出发点能直线连接目标点
/// 先从出发点尝试直达，遇到障碍，则沿两边搜索，必然遇到障碍点或拐点，如果是障碍点，则尝试直达该障碍点，如果无法直达，则会从新障碍点循环计算，直到遇到拐点。如果是拐点，则尝试直达该拐点，如果可直达，则保留该拐点为跳点。
pub fn jp_make_neighbors<N: PartialOrd + Zero + Copy, M: Map<N> + TileMap>(
    map: &mut M,
    cur: NodeIndex,
    end: NodeIndex,
    finder: &mut Finder<N, JPEntry<N>>,
) -> NodeNeighbors<N> {
    let e = &mut finder.nodes[cur.0];
    e.entry.state = NodeState::FromClose;
    let g = e.entry.g;
    let mut n = NodeNeighbors {
        f: N::zero(),
        node: cur,
        start: finder.neighbors.len(),
        end: 0,
    };
    for r in map.get_neighbors(cur, e.entry.parent) {
        let e = &mut finder.nodes[r.0];
        // 节点条目的版本一样，表示本次搜索已经处理了该节点
        if finder.version == e.entry.version {
            let g1 = map.get_g(r, cur);
            // 如果新的父节点算出来的g比原来的大，则跳过该节点
            if e.entry.g <= g + g1 {
                continue;
            }
            // 如果新的父节点算出来的g比原来的小，则更换父节点
            e.entry.g = g + g1;
            e.entry.parent = cur;
        } else {
            //新增节点条目
            e.entry.version = finder.version;
            e.entry.parent = cur;
            e.entry.state = NodeState::FromOpen;
            e.entry.g = g + map.get_g(r, cur);
            e.entry.h = map.get_h(r, end);
        }
        finder.neighbors.push(FNode {
            f: e.entry.g + e.entry.h,
            node: r,
        });
    }
    n.end = finder.neighbors.len();
    // 对节点进行排序
    if n.start < n.end {
        finder.neighbors[n.start..n.end].sort();
        n.f = finder.neighbors[n.start].f;
    }
    n
}

