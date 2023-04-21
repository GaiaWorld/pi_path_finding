//!
//! A*寻路算法--抽象实现
//! 给定一组代表全图的通用节点`nodes`，以及寻路的起始节点在`nodes`的索引`start` 和目的节点在`nodes`索引`end`进行A* 寻路；
//! 如果寻路到终点 或 寻路的结果节点数目超过`max_number`，则终止寻路并返回结果；
//! 注意：如果 from-to 的代价不对称，那么A*算法的结果可能不是最优的
//! 传统A*算法中，open堆很容易过大， 放入节点和弹出节点都会很慢
//! 优化版的实现，增加一个节点邻居NodeNeighbors，这个节点邻居代表一个节点的全部邻居，可以从中取出最优邻居节点， 将NodeNeighbors放入open表，这样可以大大缩小open表的大小。
//! 寻路时，步骤1、创建起点的节点邻居nn。 步骤2、弹出nn中的最优邻居节点node，判断是否到达，如果没有到达，则获取该node的节点邻居nn1，如果open表不为空，从表头取最优的节点邻居nn2，比较nn2、nn1和nn，选择放入nn1或nn， 获得最优的新的nn。 重复该步骤2。
//! 支持双端寻路，双端寻路一般比单端寻路速度快。如果 from-to 的代价不对称，则应该使用从起点出发的单端寻路

use num_traits::Zero;
use pi_null::Null;
use std::{
    cmp::{Ordering, Reverse},
    collections::BinaryHeap,
    fmt::Debug,
    mem,
};
use std::ops::Deref;

/// A*节点索引
#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Default, Debug)]
pub struct NodeIndex(pub usize);

impl Deref for NodeIndex {
    type Target = usize;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}


impl Null for NodeIndex {
    fn is_null(&self) -> bool {
        self.0.is_null()
    }

    fn null() -> Self {
        NodeIndex(usize::null())
    }
}
/// A*节点状态
#[derive(PartialEq, Eq, Clone)]
pub enum NodeState {
    /// 还未使用
    None,
    /// 在from的open表中
    FromOpen,
    /// 在to的open表中
    ToOpen,
    /// 在from的已搜索
    FromClose,
    /// 在to的已搜索
    ToClose,
}
/// A*算法的返回类型
#[derive(Debug)]
pub enum AStarResult {
    /// 找到了路径
    Found,
    /// 没找到路径
    NotFound,
    /// 被max_number限制，没找到路径，返回最接近的节点
    LimitNotFound(NodeIndex),
}

// A*寻路的节点条目
pub trait NodeEntry<N: PartialOrd + Zero + Copy + Debug> {
    fn g(&self) -> N;
    fn h(&self) -> N;
    fn state(&self) -> NodeState;
    fn parent(&self) -> NodeIndex;
}

// A*搜索器， 记录每个节点的搜索过程中的信息
pub struct Finder<N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    // 节点数组
    pub nodes: Vec<E>,
    // 节点邻居的邻居表
    pub neighbors: Vec<FNode<N>>,
    /// 当前的搜索版本，每次搜索加1
    pub version: usize,
}

///
/// ## A* 寻路的堆
/// ### 对`N`的约束，数字集合
/// + 算术运算，可拷贝，可偏序比较；
/// + 实际使用的时候就是数字类型，比如：i8/u8/i32/u32/f32/f64；
pub struct AStar<N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    // 从from搜to的Open表，最小堆
    from_open: BinaryHeap<Reverse<NodeNeighbors<N>>>,
    // A*搜索器
    finder: Finder<N, E>,
}

impl<N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> AStar<N, E> {
    /// 实例化A*寻路算法的结构体
    /// 必须传入最大可能的全图节点容量，可选传入调用A*算法时可能搜索的节点数量以提前初始化节约性能，但也可为0
    pub fn with_capacity(map_capacity: usize, node_number: usize) -> Self {
        let mut vec = Vec::with_capacity(map_capacity);
        vec.resize_with(map_capacity, || E::default());
        AStar {
            from_open: BinaryHeap::with_capacity(node_number),
            finder: Finder {
                nodes: vec,
                neighbors: Vec::with_capacity(node_number),
                version: 0,
            },
        }
    }
    /// 重新设置地图节点数组的大小
    pub fn resize(&mut self, map_capacity: usize) {
        self.finder.nodes.resize_with(map_capacity, || E::default())
    }
    /// 从起点向终点的A*寻路算法
    /// 给定一组代表全图的通用节点`nodes`，以及寻路的起始节点在`nodes`的索引`start` 和目的节点在`nodes`索引`end`进行A* 寻路；
    /// 如果 寻路到终点 或 寻路的结果节点数目超过`max_number`，则终止寻路并返回结果；
    ///
    /// 返回结果：是否成功寻路至目标点
    ///
    /// 大概流程：
    ///
    /// + 1. 创建起点的节点邻居nn。
    ///
    /// + 2、弹出nn中的最优邻居节点node，判断是否到达，如果没有到达，则获取该node的节点邻居nn1，如果open表不为空，从表头取最优的节点邻居nn2，比较nn2、nn1和nn，选择放入nn1或nn， 获得最优的新的nn。 重复该步骤2。
    ///
    /// + 3、如果open表为空，则返回NotFound。
    ///
    pub fn find<Arg>(
        &mut self,
        start: NodeIndex,
        end: NodeIndex,
        max_number: usize,
        arg: &mut Arg,
        make_neighbors: fn(&mut Arg, NodeIndex, NodeIndex, &mut Finder<N, E>) -> NodeNeighbors<N>,
    ) -> AStarResult {
        self.from_open.clear();
        self.finder.neighbors.clear();
        self.finder.version += 1;
        // 创建起点的节点邻居
        let mut nn = make_neighbors(arg, start, end, &mut self.finder);
        loop {
            // 弹出nn中的最优邻居节点NodeIndex，弹出后，nn代价大小会变
            let node = nn.pop(&self.finder, NodeState::FromOpen);
            // 判断节点是否无效
            if node.is_null() {
                // 无效则继续弹出新的nn
                if let Some(n) = self.from_open.pop() {
                    nn = n.0;
                } else {
                    // 如果open表空， 则返回NotFound
                    return AStarResult::NotFound;
                }
                continue;
            }
            // 判断是否已经找到路径
            if node == end {
                return AStarResult::Found;
            }
            // 如果nn有效
            if nn.start < nn.end {
                // 创建该点的节点邻居
                let mut nn1 = make_neighbors(arg, node, end, &mut self.finder);
                // 如果该节点邻居可用
                if nn1.start < nn1.end {
                    // 如果nn1优于nn， 则交换两者
                    if nn1 < nn {
                        nn1 = mem::replace(&mut nn, nn1);
                    }
                    // 将nn1放入堆上
                    self.from_open.push(Reverse(nn1));
                    // 如果超过max_number，则返回LimitNotFound
                    if self.from_open.len() > max_number {
                        return AStarResult::LimitNotFound(node);
                    }
                }
            } else {
                // 如果nn无效则用新的nn
                nn = make_neighbors(arg, node, end, &mut self.finder);
                continue;
            }
            // 用堆上的nn2和nn比较
            if let Some(n) = self.from_open.peek() {
                // 如果堆上的nn2优于nn
                if n.0 < nn {
                    // 取出nn2并交换两者
                    let nn2 = self.from_open.pop().unwrap().0;
                    // 将nn放入到堆上
                    self.from_open.push(Reverse(nn));
                    nn = nn2;
                }
            }
        }
    }
    /// 返回结果的迭代器
    pub fn result_iter<'a>(&'a self, node: NodeIndex) -> ResultIterator<'a, N, E> {
        ResultIterator {
            finder: &self.finder,
            node,
        }
    }
}

// 结果的迭代器
#[derive(Clone)]
pub struct ResultIterator<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    pub finder: &'a Finder<N, E>,
    pub node: NodeIndex,
}

impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> Iterator
    for ResultIterator<'a, N, E>
{
    type Item = NodeIndex;
    fn next(&mut self) -> Option<Self::Item> {
        if self.node.is_null() {
            return None;
        }
        let old = self.node;
        let e = &self.finder.nodes[self.node.0];
        self.node = e.parent();
        Some(old)
    }
}

/// 节点邻居
#[derive(Debug, Clone)]
pub struct NodeNeighbors<N: PartialOrd + Zero + Copy + Debug> {
    pub f: N,
    pub node: NodeIndex,
    pub start: usize,
    pub end: usize,
}

impl<N: PartialOrd + Zero + Copy + Debug> NodeNeighbors<N> {
    /// 从指定的节点邻居上获取其的最优邻居节点
    fn pop<E: NodeEntry<N> + Default>(
        &mut self,
        finder: &Finder<N, E>,
        state: NodeState,
    ) -> NodeIndex {
        while self.start < self.end {
            let n = &finder.neighbors[self.start];
            let e = &finder.nodes[n.node.0];
            self.start += 1;
            // 要求该节点的parnet必须是节点邻居上的节点
            if e.state() == state && e.parent() == self.node {
                // 更新节点邻居的f值
                while self.start < self.end {
                    let s = &finder.neighbors[self.start];
                    let e = &finder.nodes[s.node.0];
                    if e.state() == state && e.parent() == self.node {
                        self.f = e.g() + e.h();
                        break;
                    }
                    self.start += 1;
                }
                return n.node;
            }
        }
        NodeIndex(usize::MAX)
    }
}
// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Debug> Eq for NodeNeighbors<N> {}
// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Debug> PartialEq for NodeNeighbors<N> {
    fn eq(&self, other: &Self) -> bool {
        self.f.eq(&other.f)
    }
}
// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Debug> PartialOrd for NodeNeighbors<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.f.partial_cmp(&other.f)
    }
}
// 通过f的比较实现Ord trait，以能在堆中排序
impl<N: PartialOrd + Zero + Copy + Debug> Ord for NodeNeighbors<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f.partial_cmp(&other.f) {
            None => self.node.cmp(&other.node),
            Some(r) => r,
        }
    }
}

/// 排序节点
#[derive(Clone)]
pub struct FNode<N: PartialOrd + Zero + Copy + Debug> {
    pub f: N,
    pub node: NodeIndex,
}

// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Debug> Eq for FNode<N> {}
// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Debug> PartialEq for FNode<N> {
    fn eq(&self, other: &Self) -> bool {
        self.f.eq(&other.f)
    }
}
// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Debug> PartialOrd for FNode<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.f.partial_cmp(&other.f)
    }
}
// 通过f的比较实现Ord trait，以能在数组中排序
impl<N: PartialOrd + Zero + Copy + Debug> Ord for FNode<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f.partial_cmp(&other.f) {
            None => self.node.cmp(&other.node),
            Some(r) => r,
        }
    }
}

///
/// ## 双端 A* 寻路的堆
/// ### 对`N`的约束，数字集合
/// + 算术运算，可拷贝，可偏序比较；
/// + 实际使用的时候就是数字类型，比如：i8/u8/i32/u32/f32/f64；
pub struct DualAStar<N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    // 从from搜to的AStar
    astar: AStar<N, E>,
    // 从to搜from的Open表，最小堆
    to_open: BinaryHeap<NodeNeighbors<N>>,
}
