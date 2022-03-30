//!
//! # A*算法--抽象实现
//!
//!http://ser.yinengyun.com:8181/docs/pi_ai/pi_ai-1cmdeekhb57es
//!
use heap::slab_heap::SlabHeap;
use num_traits::Zero;
use std::cmp::{Ordering, PartialOrd};

/// ## A* 寻路的抽象寻路主体代理约束
///
/// 通过代理将不同寻路能力主体的寻路情况交由上层逻辑决定，增加导航图的通用性及寻路能力的动态性
///
/// 需要实现主体在两点间是否可达、及两点间的代价
///
/// ### 对`N`的约束，连加减乘除都不需要的数字集合
///
/// + 算术运算，可拷贝，可偏序比较；
/// + 实际使用的时候就是数字类型，比如：i8/u8/i32/u32/f32/f64；
/// + TODO 注意：如果 from-to的代价不对称，那么A*算法的结果可能不是最优的，有待寻求理论证明？
pub trait AFilter<N>
where
    N: PartialOrd + Zero + Copy,
{
    /// 从`from`节点到`to`节点是否可达
    fn is_pass(&self, from: usize, to: usize) -> bool;

    /// 从`parent`节点到当前`cur`节点的`g`，即从父节点到当前节点的实际代价
    fn get_g(&self, cur: usize, parent: usize) -> N;

    /// 从当前`cur`节点到`end`节点的`h`，即从当前节点到终点的预估代价
    fn get_h(&self, cur: usize, end: usize) -> N;
}

/// ## A* 寻路的抽象地图
///
/// 需要实现获取遍历邻居节点的迭代器的方法， 其中迭代器`Item`指定为`usize`，表示节点在全图节点集合的索引
///
pub trait AStarMap<'a> {
    /// 迭代器的关联类型，指定了迭代器`Item`为`usize`
    type AStartNodeIter: Iterator<Item = usize>;

    /// 获取遍历邻居节点的迭代器
    fn get_neighbors(&'a self, cur: usize, parent: Option<usize>) -> Self::AStartNodeIter;
}

///
/// ## A* 寻路算法的结构体
///
/// + 使用结构体以复用`open`,`close`,`path`等数据结构
/// + 返回路径位于结构体内
///
/// ### 对`N`的约束，连加减乘除都不需要的数字集合
///
/// + 算术运算，可拷贝，可偏序比较；
/// + 实际使用的时候就是数字类型，比如：i8/u8/i32/u32/f32/f64；
pub struct AStar<N>
where
    N: PartialOrd + Zero + Copy,
{
    // Open表，用了最小堆
    open: SlabHeap<OpenHeapIteam<N>>,

    // 所有节点的内部结构
    impl_nodes: Vec<ANodeImpl<N>>,

    /// path路径，Vec的值是节点在全图节点集合的索引
    pub result_indies: Vec<usize>,
}

impl<N> AStar<N>
where
    N: PartialOrd + Zero + Copy,
{
    ///
    /// 实例化A*寻路算法的结构体
    ///
    /// 传入调用A*算法时预估的全图节点数量，以提前初始化节约性能，但也可为0
    ///
    /// ### 对`N`的约束，连加减乘除都不需要的数字集合
    ///
    /// + 算术运算，可拷贝，可偏序比较；
    /// + 实际使用的时候就是数字类型，比如：i8/u8/i32/u32/f32/f64；
    pub fn new(nodes_num: usize) -> AStar<N> {
        let mut result = AStar::<N> {
            open: SlabHeap::with_capacity(nodes_num / 10, Ordering::Less),
            impl_nodes: Vec::with_capacity(nodes_num / 10),
            result_indies: Vec::with_capacity(nodes_num / 10),
        };
        for _ in 0..nodes_num {
            result.impl_nodes.push(ANodeImpl::default());
        }
        result
    }

    ///
    /// 通用的A*寻路算法
    ///
    /// 给定一组代表全图的通用节点`nodes`，以及寻路的起始节点在`nodes`的索引`start` 和目的节点在`nodes`索引`end`进行A* 寻路；
    ///
    /// 如果 寻路到终点 或 寻路的结果节点数目超过`max_nodes`，则终止寻路并返回结果；
    ///
    /// 返回结果：是否成功寻路至目标点
    ///
    /// 该函数类型返回的约束，见：`AstarNode` 描述
    ///
    /// 大概流程：
    ///
    ///首先将起始结点S放入OPEN表，CLOSE表置空，算法开始时：
    ///
    /// + 1、如果OPEN表不为空，从表头取一个结点n，如果为空算法失败,失败返回CLOSE表中估价值`f(n)`最小的节点。
    ///
    /// + 2、n是目标解吗？是，找到一个解（继续寻找，或终止算法）。
    ///
    /// + 3、将n的所有后继结点展开，就是从n可以直接关联的结点（子结点），如果不在CLOSE表中，就将它们放入OPEN表，并把S放入CLOSE表，同时计算每一个后继结点的估价值`f(n)`，将OPEN表按f(x)排序，最小的放在表头，重复算法，回到1。
    ///
    pub fn find_path<'a, Map, Filter>(
        &mut self,
        map: &'a Map,
        start: usize,
        end: usize,
        filter: &Filter,
        max_nodes: usize,
    ) -> bool
    where
        Map: AStarMap<'a>,
        Filter: AFilter<N>,
    {
        self.open.clear();
        self.result_indies.clear();
        for item in self.impl_nodes.iter_mut() {
            item.set_status(NodeStatus::None);
        }

        self.impl_nodes[start].init(start, end, filter);

        let mut distance_to_end = self.impl_nodes[start].h; // 距离目标的最近距离
        let mut nearest = start; // 距离目标的最近点，存nodes中的index

        let mut success = false; //是否可到达终点

        self.impl_nodes[start].set_status(NodeStatus::Opened);
        let mut best_next: Option<NodeIndex> = Some(start);

        // 当前节点的可行邻居节点
        let mut neibors = Vec::with_capacity(8);

        let mut visit_node_count = 1;

        while self.open.len() != 0 || !best_next.is_none() {
            let cur_node_idx = match best_next {
                None => {
                    // 取最小的f的节点index出来
                    let cur_node = self.open.pop().expect("Empty Open List");
                    cur_node.index
                }
                Some(idx) => {
                    best_next = None;
                    idx
                }
            };
            // 已经找到目标点，退出循环
            if end == cur_node_idx {
                nearest = end;
                success = true;
                self.impl_nodes[cur_node_idx].set_status(NodeStatus::Closed);
                break;
            }

            let cur_node = unsafe { self.impl_nodes.get_unchecked(cur_node_idx) };
            let cur_node_g = cur_node.g;

            if !cur_node.next_index.is_none() {
                // 最优邻居出堆了，将下一优邻居进入open
                let next_idx = cur_node.next_index.unwrap();
                let mut next = unsafe { self.impl_nodes.get_unchecked_mut(next_idx) };
                if next.status == NodeStatus::Linked {
                    next.set_status(NodeStatus::Opened);
                    next.open_index = self.open.push(OpenHeapIteam {
                        f: next.f,
                        index: next_idx,
                    }); // 将新点进入open表
                }
            }

            let cur_node = unsafe { self.impl_nodes.get_unchecked(cur_node_idx) };
            let distance = cur_node.h;
            // 如果当前点离终点更近，则记住当前点
            if distance < distance_to_end {
                nearest = cur_node_idx;
                distance_to_end = distance;
            }

            let parent = match cur_node.parent_index {
                None => None,
                Some(index) => Some(index),
            };
            neibors.clear();
            for neighbor in map.get_neighbors(cur_node_idx, parent) {
                // 不可走，忽略
                if !filter.is_pass(cur_node_idx, neighbor) {
                    continue;
                }

                // 邻居即父节点，忽略
                if !parent.is_none() && parent.unwrap() == neighbor {
                    continue;
                }
                let node = unsafe { self.impl_nodes.get_unchecked_mut(neighbor) };
                let mut g = N::zero();
                if node.status != NodeStatus::None {
                    // 邻居以及被close，忽略
                    if node.status == NodeStatus::Closed {
                        continue;
                    }

                    ////邻居已经被open或被link，则对比g是否需要刷新父节点
                    if node.status == NodeStatus::Linked || node.status == NodeStatus::Opened {
                        g = cur_node_g + filter.get_g(neighbor, cur_node_idx);
                        if g > node.g {
                            continue;
                        }
                    }
                } else {
                    g = cur_node_g + filter.get_g(neighbor, cur_node_idx);
                    node.init(neighbor, end, filter);
                    node.set_status(NodeStatus::Linked);
                    visit_node_count += 1;
                }
                node.reset_parent(cur_node_idx, g);
                neibors.push(neighbor);
            }

            neibors.sort_unstable_by(|a, b| {
                let a_f = unsafe { self.impl_nodes.get_unchecked(*a).f };
                let b_f = unsafe { self.impl_nodes.get_unchecked(*b).f };
                if a_f < b_f {
                    Ordering::Less
                } else if a_f > b_f {
                    Ordering::Greater
                } else {
                    Ordering::Equal
                }
            });

            // 将所有邻居节点按照f值排序连接起来
            let len = neibors.len();
            for idx in 0..len {
                let node = unsafe { self.impl_nodes.get_unchecked_mut(neibors[idx]) };
                let node_last = node.last_index;
                let node_next = node.next_index;
                node.reset_link(
                    if idx + 1 >= len {
                        None
                    } else {
                        Some(neibors[idx + 1])
                    },
                    if idx as isize - 1 < 0 {
                        None
                    } else {
                        Some(neibors[idx - 1])
                    },
                );
                if node.status == NodeStatus::Opened {
                    self.open.update(node.open_index, Ordering::Less);
                }
                if !node_last.is_none() {
                    unsafe { self.impl_nodes.get_unchecked_mut(node_last.unwrap()) }
                        .set_next(node_next);
                };
                if !node_next.is_none() {
                    unsafe { self.impl_nodes.get_unchecked_mut(node_next.unwrap()) }
                        .set_last(node_last);
                };
            }

            if neibors.len() > 0 {
                // 只open最优邻居
                let mut best_neighbor = unsafe { self.impl_nodes.get_unchecked_mut(neibors[0]) };
                best_neighbor.set_status(NodeStatus::Opened);
                let open_top = self.open.get_top();
                if open_top.is_none() || best_neighbor.f <= open_top.unwrap().f {
                    best_next = Some(neibors[0]);
                } else {
                    best_neighbor.open_index = self.open.push(OpenHeapIteam {
                        f: best_neighbor.f,
                        index: neibors[0],
                    }); // 将新点进入open表
                }
            }
            unsafe { self.impl_nodes.get_unchecked_mut(cur_node_idx) }
                .set_status(NodeStatus::Closed);

            // 如果已找的点太多，则退出循环
            if visit_node_count >= max_nodes {
                break;
            }
        }

        // TODO: 优化，可不可以少些遍历，目前没想到；

        // 从终点开始往回溯
        let mut anode_ref = self.impl_nodes.get(nearest).expect("Wrong ANode Ref");

        self.result_indies.push(nearest);

        while anode_ref.parent_index != None {
            self.result_indies.push(anode_ref.parent_index.unwrap());

            anode_ref = self
                .impl_nodes
                .get(anode_ref.parent_index.unwrap())
                .unwrap();
        }

        // 反转，因为需要的是从起点到终点的路径
        self.result_indies.reverse();
        success
    }
}

// =================================== 本地

// 节点数组的索引
type NodeIndex = usize;

// Open表的索引
type OpenKey = usize;

#[derive(PartialEq, Eq, Clone)]
enum NodeStatus {
    None,
    Linked,
    Opened,
    Closed,
}

// A*算法的可行节点
struct ANodeImpl<N: PartialOrd + Zero + Copy> {
    g: N, // 起点到该点的真实代价
    h: N, // 该点到终点的估算
    f: N, // f = g + h
    status: NodeStatus,
    parent_index: Option<NodeIndex>, // 父节点在nodes中的index
    next_index: Option<NodeIndex>,
    last_index: Option<NodeIndex>,
    open_index: OpenKey,
}

// 通过f的比较实现ANode的Ord trait，以能在最小堆中排序
// TODO: 当前，若f值为nan，返回结果为相等，存在隐患需要解决
impl<N: PartialOrd + Zero + Copy> Ord for ANodeImpl<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f.partial_cmp(&other.f) {
            None => Ordering::Equal,
            Some(r) => r,
        }
    }
}

// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Copy> Eq for ANodeImpl<N> {}

// Ord trait所需
impl<N: PartialOrd + Zero + Copy> PartialEq for ANodeImpl<N> {
    fn eq(&self, other: &Self) -> bool {
        self.f.eq(&other.f)
    }
}

// Ord trait所需
impl<N: PartialOrd + Zero + Copy> PartialOrd for ANodeImpl<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.f.partial_cmp(&other.f)
    }
}

impl<N: PartialOrd + Zero + Copy> ANodeImpl<N> {
    #[inline]
    fn default() -> ANodeImpl<N> {
        ANodeImpl {
            g: N::zero(),
            h: N::zero(),
            f: N::zero(),
            parent_index: None,
            next_index: None,
            last_index: None,
            status: NodeStatus::None,
            open_index: 0,
        }
    }

    // 通过Node获取新的内部ANode
    fn init<Filter: AFilter<N>>(&mut self, src: NodeIndex, end: NodeIndex, filter: &Filter) {
        let h = filter.get_h(src, end);
        self.g = N::zero();
        self.h = h;
        self.f = h;
        self.parent_index = None;
        self.next_index = None;
        self.last_index = None;
        self.status = NodeStatus::None;
    }

    #[inline]
    fn reset_parent(&mut self, parent_node_index: NodeIndex, new_g: N) {
        self.g = new_g;
        self.f = new_g + self.h;
        self.parent_index = Some(parent_node_index);
    }

    #[inline]
    fn reset_link(&mut self, next_index: Option<NodeIndex>, last_index: Option<NodeIndex>) {
        self.set_next(next_index);
        self.set_last(last_index);
    }

    #[inline]
    fn set_next(&mut self, next_index: Option<NodeIndex>) {
        self.next_index = next_index;
    }

    #[inline]
    fn set_last(&mut self, last_index: Option<NodeIndex>) {
        self.last_index = last_index;
    }

    #[inline]
    fn set_status(&mut self, status: NodeStatus) {
        self.status = status;
    }
}

struct OpenHeapIteam<N: PartialOrd + Zero + Copy> {
    f: N,
    index: NodeIndex,
}

impl<N: PartialOrd + Zero + Copy> Ord for OpenHeapIteam<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.f.partial_cmp(&other.f) {
            None => Ordering::Equal,
            Some(r) => r,
        }
    }
}

impl<N: PartialOrd + Zero + Copy> PartialOrd for OpenHeapIteam<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.f.partial_cmp(&other.f)
    }
}

// Ord trait所需
impl<N: PartialOrd + Zero + Copy + Copy> Eq for OpenHeapIteam<N> {}

// Ord trait所需
impl<N: PartialOrd + Zero + Copy> PartialEq for OpenHeapIteam<N> {
    fn eq(&self, other: &Self) -> bool {
        self.f.eq(&other.f)
    }
}
