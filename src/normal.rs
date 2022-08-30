//!
//! 标准A*算法需要的节点条目和创建节点邻居的函数

use std::{cmp::PartialOrd, fmt::Debug};

use num_traits::Zero;

use crate::*;

/// ## A*寻路的抽象地图
///
/// 需要实现获取遍历邻居节点的迭代器的方法， 其中迭代器`Item`指定为`NodeIndex`，表示节点在全图节点集合的索引
///
pub trait Map<N>
where
    N: PartialOrd + Zero + Copy + Debug,
{
    /// 迭代器的关联类型，指定了迭代器`Item`为`NodeIndex`
    type NodeIter: Iterator<Item = NodeIndex>;

    /// 获取遍历邻居节点的迭代器
    fn get_neighbors(&self, cur: NodeIndex, parent: NodeIndex) -> Self::NodeIter;
    /// 从`parent`节点到当前`cur`节点的`g`，即从父节点到当前节点的实际代价
    fn get_g(&self, cur: NodeIndex, parent: NodeIndex) -> N;
    /// 从当前`cur`节点到`end`节点的`h`，即从当前节点到终点的预估代价
    fn get_h(&self, cur: NodeIndex, end: NodeIndex) -> N;
}

/// 节点条目
pub struct Entry<N: PartialOrd + Zero + Copy + Debug> {
    pub g: N,
    pub h: N,
    pub version: usize,
    pub parent: NodeIndex,
    pub state: NodeState,
}
impl<N: PartialOrd + Zero + Copy + Debug> Default for Entry<N> {
    fn default() -> Self {
        Entry {
            g: N::zero(),
            h: N::zero(),
            version: 0,
            parent: NodeIndex(usize::MAX),
            state: NodeState::None,
        }
    }
}
impl<N: PartialOrd + Zero + Copy + Debug> NodeEntry<N> for Entry<N> {
    fn g(&self) -> N {
        self.g
    }
    fn h(&self) -> N {
        self.h
    }
    fn state(&self) -> NodeState {
        self.state.clone()
    }
    fn parent(&self) -> NodeIndex {
        self.parent
    }
}

/// 创建节点邻居的函数
pub fn make_neighbors<N: PartialOrd + Zero + Copy + Debug, M: Map<N>>(
    map: &mut M,
    cur: NodeIndex,
    end: NodeIndex,
    finder: &mut Finder<N, Entry<N>>,
) -> NodeNeighbors<N> {
    let e = &mut finder.nodes[cur.0];
    e.state = NodeState::FromClose;
    let g = e.g;
    let mut n = NodeNeighbors {
        f: N::zero(),
        node: cur,
        start: finder.neighbors.len(),
        end: 0,
    };
    for r in map.get_neighbors(cur, e.parent) {
        let e = &mut finder.nodes[r.0];
        // 节点条目的版本一样，表示本次搜索已经处理了该节点
        if finder.version == e.version {
            let g1 = map.get_g(r, cur);
            // 如果新的父节点算出来的g比原来的大，则跳过该节点
            if e.g <= g + g1 {
                continue;
            }
            // 如果新的父节点算出来的g比原来的小，则更换父节点
            e.g = g + g1;
            e.parent = cur;
        } else {
            //新增节点条目
            e.version = finder.version;
            e.parent = cur;
            e.state = NodeState::FromOpen;
            e.g = g + map.get_g(r, cur);
            e.h = map.get_h(r, end);
        }
        finder.neighbors.push(FNode {
            f: e.g + e.h,
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
