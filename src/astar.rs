//!
//! # A*算法--抽象实现
//!
use heap::slab_heap::SlabHeap;
use num_traits::Zero;
use std::cmp::{Ordering, PartialOrd};
use std::collections::HashMap;

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
    open: SlabHeap<ANodeImpl<N>>,

    // Close表：Key=Node索引
    close: HashMap<NodeIndex, ANodeImpl<N>>,

    // nodes中的index 到 open-heap 的 index的索引
    node_open_map: HashMap<NodeIndex, OpenKey>,

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
        let nodes_num = nodes_num / 10;

        AStar::<N> {
            open: SlabHeap::with_capacity(nodes_num, Ordering::Less),

            close: HashMap::with_capacity(nodes_num),

            node_open_map: HashMap::with_capacity(nodes_num),

            result_indies: Vec::with_capacity(nodes_num),
        }
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
        self.close.clear();
        self.node_open_map.clear();
        self.result_indies.clear();

        let anode = ANodeImpl::new(start, end, None, filter);

        let mut distance_to_end = anode.h; // 距离目标的最近距离

        let mut nearest = start; // 距离目标的最近点，存nodes中的index

        let mut success = false; //是否可到达终点

        let open_index = self.open.push(anode); // 将开始点扔到open表

        self.node_open_map.insert(start, open_index);

        while self.open.len() != 0 {
            let cur_node = self.open.pop().expect("Empty Open List"); // 取最小的f的节点index出来

            self.node_open_map.remove(&cur_node.src_index);

            // 已经找到目标点，退出循环
            if end == cur_node.src_index {
                nearest = end;
                success = true;

                self.close.insert(cur_node.src_index, cur_node);
                break;
            }

            let distance = cur_node.h;

            // 如果当前点离终点更近，则记住当前点
            if distance < distance_to_end {
                nearest = cur_node.src_index;
                distance_to_end = distance;
            }

            let parent = match cur_node.parent_index {
                None => None,
                Some(index) => Some(index),
            };

            // 遍历邻居
            for neighbor in map.get_neighbors(cur_node.src_index, parent) {
                let neighbor = neighbor;

                // 不可走，下一个邻居
                if !filter.is_pass(cur_node.src_index, neighbor) {
                    continue;
                }

                if self.close.contains_key(&neighbor) {
                    // 该节点已经close了
                    continue;
                } else {
                    match self.node_open_map.get(&neighbor) {
                        None => {
                            // 新节点
                            let anode = ANodeImpl::new(neighbor, end, Some(&cur_node), filter);

                            let open_index = self.open.push(anode); // 将新点进入open表

                            self.node_open_map.insert(neighbor, open_index);
                        }
                        Some(neighbor_heap_index) => {
                            // 已经open的节点，判断并更新代价
                            let g_from_cur = filter.get_g(cur_node.src_index, neighbor);

                            let g = cur_node.g + g_from_cur;
                            let h = filter.get_h(neighbor, end);

                            let neighbor_ref = self.open.get_mut(*neighbor_heap_index).unwrap();

                            // 只有 新路径代价 小于 已有 路径时，才需要更新代价和堆
                            if neighbor_ref.f <= g + h {
                                continue;
                            }

                            // 更新父节点、g、f
                            neighbor_ref.reset_parent(cur_node.src_index, g);

                            // 刷新最小堆
                            self.open.update(*neighbor_heap_index, Ordering::Less);
                        }
                    }
                }
            }

            self.close.insert(cur_node.src_index, cur_node); // 当前点close

            // 如果已找的点太多，则退出循环
            if self.open.len() + self.close.len() >= max_nodes {
                break;
            }
        }

        // TODO: 优化，可不可以少些遍历，目前没想到；

        // 从终点开始往回溯
        let mut anode_ref = self.close.get(&nearest).expect("Wrong ANode Ref");

        self.result_indies.push(anode_ref.src_index);

        while anode_ref.parent_index != None {
            self.result_indies.push(anode_ref.parent_index.unwrap());

            anode_ref = self.close.get(&anode_ref.parent_index.unwrap()).unwrap();
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

// A*算法的可行节点
struct ANodeImpl<N: PartialOrd + Zero + Copy> {
    g: N, // 起点到该点的真实代价
    h: N, // 该点到终点的估算
    f: N, // f = g + h

    src_index: NodeIndex,            // 原数据数组nodes的index
    parent_index: Option<NodeIndex>, // 父节点在nodes中的index
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
    // 通过Node获取新的内部ANode
    fn new<Filter: AFilter<N>>(
        src: NodeIndex,
        end: NodeIndex,
        parent_node: Option<&ANodeImpl<N>>,
        filter: &Filter,
    ) -> ANodeImpl<N> {
        match parent_node {
            None => {
                let h = filter.get_h(src, end);
                ANodeImpl {
                    g: N::zero(),
                    h,
                    f: h,
                    src_index: src,
                    parent_index: None,
                }
            }
            Some(parent_node) => {
                let g = filter.get_g(parent_node.src_index, src) + parent_node.g;
                let h = filter.get_h(src, end);
                ANodeImpl {
                    g,
                    h,
                    f: g + h,
                    src_index: src,
                    parent_index: Some(parent_node.src_index),
                }
            }
        }
    }

    // 更新父节点及g、h
    fn reset_parent(
        &mut self,
        parent_node_index: NodeIndex, //parent node在nodes中的index
        new_g: N,
    ) {
        self.parent_index = Some(parent_node_index);
        self.g = new_g;
        self.f = new_g + self.h;
    }
}
