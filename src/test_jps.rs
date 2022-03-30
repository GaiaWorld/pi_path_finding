//!
//! # JPS算法的新的尝试
//!
//! + 该方法能找到路径，但不保证为最优解
use crate::{AJPSFilter, AJPSStar, AJPSStarMap};
use nalgebra::{Point2, Scalar};
use num_derive::FromPrimitive;
use num_traits::{cast::AsPrimitive, FromPrimitive, Num};

/// ## 方格JPS 寻路的方格图
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
///
pub struct TestJPSMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    // 该图所有节点
    nodes: Vec<TestJPSNode>,
    // 该图最大行数
    max_row: usize,
    // 该图最大列数
    max_column: usize,
    // 该图节点的尺寸
    cell_size: N,
    // 该图起点位于场景中的坐标
    start_pos: Point2<N>,
    // 调用A*的内部存储结构,支持多线程（一张图同时多个寻路请求）
    astar: Vec<AJPSStar<N>>,
    //目标点的行
    target_row: usize,
    //目标点的列
    target_column: usize,
}

impl<N> TestJPSMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    ///
    /// 新建一个方格图
    ///
    /// 需指定方格图的行数和列数，每个方格的边长，以及全图在场景中的起始坐标
    ///
    /// ### 对`N`的约束
    ///
    /// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
    /// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
    ///
    pub fn new_map(
        row: usize,
        column: usize,
        cell_size: N,
        start_pos: Point2<N>,
        max_find_num_in_time: usize,
    ) -> TestJPSMap<N> {
        let mut map = TestJPSMap {
            nodes: Vec::with_capacity(row * column),
            max_row: row - 1,
            max_column: column - 1,
            cell_size: cell_size,
            start_pos: start_pos,
            astar: Vec::with_capacity(max_find_num_in_time),
            target_column: 0,
            target_row: 0,
        };
        for _ in 0..max_find_num_in_time {
            map.astar.push(AJPSStar::new(row * column));
        }
        let mut cur_row = 0;
        while cur_row < row {
            let mut cur_column = 0;
            while cur_column < column {
                let node = TestJPSNode {
                    row: cur_row,
                    column: cur_column,
                    index: cur_row * (map.max_column + 1) + cur_column,
                    is_block: false,
                    is_target: false,
                };
                map.nodes.push(node);
                cur_column = cur_column + 1;
            }
            cur_row = cur_row + 1;
        }

        map
    }

    ///
    ///设置方格图中指定区域的阻挡属性
    ///
    /// 需要指定区域范围，即区域最左下点及最右上点的坐标
    ///
    /// ### 对`N`的约束
    ///
    /// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
    /// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
    ///
    pub fn set_block(&mut self, min_pos: Point2<N>, max_pos: Point2<N>, block: bool) {
        let min_node_index = self.get_node_by_position(min_pos);
        let max_node_index = self.get_node_by_position(max_pos);
        let mut index = 0;
        while index < self.nodes.len() {
            if self.nodes[index].row >= self.nodes[min_node_index].row
                && self.nodes[index].row < self.nodes[max_node_index].row
            {
                if self.nodes[index].column >= self.nodes[min_node_index].column
                    && self.nodes[index].column < self.nodes[max_node_index].column
                {
                    self.nodes[index].is_block = block;
                }
            }
            index += 1;
        }
    }

    ///
    /// 通过A*算法获取寻路路径
    ///
    /// 需要指定场景中的开始坐标及目标坐标，寻路主体代理实现，以及最多访问节点的数量
    ///
    /// 返回为有效路径坐标序列
    ///
    /// + 超出方格图范围的开始点及目标点会被强行拉至方格图范围内
    ///
    /// ### 对`N`的约束
    ///
    /// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
    /// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
    ///
    pub fn get_path(
        &mut self,
        start_pos: Point2<N>,
        end_pos: Point2<N>,
        max_nodes: usize,
    ) -> Vec<Point2<N>> {
        let start_node_index = self.get_node_by_position(start_pos);
        let end_node_index = self.get_node_by_position(end_pos);

        self.nodes[end_node_index].is_target = true;
        self.target_row = self.nodes[end_node_index].row;
        self.target_column = self.nodes[end_node_index].column;

        let filter = TestJPSFilter { map_ref: self };
        let mut astar = if self.astar.len() > 0 {
            self.astar.pop().unwrap()
        } else {
            AJPSStar::new(self.nodes.len())
        };
        astar.find_path(self, start_node_index, end_node_index, &filter, max_nodes);
        self.nodes[end_node_index].is_target = false;
        self.smooth_path(&mut astar);
        // 路径转化为坐标
        let mut path = Vec::with_capacity(astar.result_indies.len());
        for &node_index in astar.result_indies.iter() {
            let node = self.nodes.get(node_index).expect("Invalid Path Node");
            let x: N = FromPrimitive::from_usize(node.column).unwrap();
            let y: N = FromPrimitive::from_usize(node.row).unwrap();
            let half: N = FromPrimitive::from_f32(0.5).unwrap();
            path.push(Point2::new(
                (x + half) * self.cell_size + self.start_pos.x,
                (y + half) * self.cell_size + self.start_pos.y,
            ));
        }
        self.astar.push(astar);
        path
    }
}

// 遍历邻居节点（即跳点）的迭代器
pub struct ATestJPSNodeIterator {
    // 邻居节点（即跳点）的集合
    neighbors: Vec<usize>,

    //当前遍历到的index
    cur_index: usize,
}

// =================================== 本地

//对astar的寻路主体代理的封装
struct TestJPSFilter<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    map_ref: *const TestJPSMap<N>,
}

impl<N> AJPSFilter<N> for TestJPSFilter<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    // 从`from`节点到`to`节点是否可达
    //所有获取的邻居都为跳点，都是可达的
    fn is_pass(&self, from: usize, to: usize) -> bool {
        let map = unsafe { self.map_ref.as_ref().unwrap() };
        map.check_cross_walkable(&map.nodes[from], &map.nodes[to])
    }

    // 从`parent`节点到当前`cur`节点的`g`，即从父节点到当前节点的实际代价
    fn get_g(&self, from: usize, to: usize) -> N {
        let map = unsafe { self.map_ref.as_ref().unwrap() };
        let to_column = to % (map.max_column + 1);
        let to_row = to / (map.max_column + 1);
        let from_column = from % (map.max_column + 1);
        let from_row = from / (map.max_column + 1);
        let max_x = to_row.max(from_row);
        let min_x = to_row.min(from_row);
        let max_y = to_column.max(from_column);
        let min_y = to_column.min(from_column);
        let temp_min = (max_x - min_x).min(max_y - min_y);
        let temp_max = (max_x - min_x).max(max_y - min_y);

        let temp: N = FromPrimitive::from_f32(1.4142135).unwrap(); //根号2
        let temp_min: N = FromPrimitive::from_usize(temp_min).unwrap();
        let temp_max: N = FromPrimitive::from_usize(temp_max).unwrap();
        temp_min * temp + temp_max - temp_min
    }

    // 从当前`cur`节点到`end`节点的`h`，即从当前节点到终点的预估代价
    fn get_h(&self, from: usize, to: usize) -> N {
        let map = unsafe { self.map_ref.as_ref().unwrap() };
        let to_column = to % (map.max_column + 1);
        let to_row = to / (map.max_column + 1);
        let from_column = from % (map.max_column + 1);
        let from_row = from / (map.max_column + 1);
        let max_x = to_row.max(from_row);
        let min_x = to_row.min(from_row);
        let max_y = to_column.max(from_column);
        let min_y = to_column.min(from_column);
        let temp_min = (max_x - min_x).min(max_y - min_y);
        let temp_max = (max_x - min_x).max(max_y - min_y);

        let temp: N = FromPrimitive::from_f32(1.4142135).unwrap(); //根号2
        let temp_min: N = FromPrimitive::from_usize(temp_min).unwrap();
        let temp_max: N = FromPrimitive::from_usize(temp_max).unwrap();
        temp_min * temp + temp_max - temp_min
    }
}

impl<N> TestJPSMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    //获取给定位置所对应的节点下标
    fn get_node_by_position(&self, pos: Point2<N>) -> usize {
        let mut column = ((pos.x - self.start_pos.x) / self.cell_size).as_();
        let mut row = ((pos.y - self.start_pos.y) / self.cell_size).as_();

        // 超出地图边界的处理
        row = row.clamp(0, self.max_row);
        column = column.clamp(0, self.max_column);

        row * (self.max_column + 1) + column
    }

    // 路径平滑 Floyd
    // https://zhuanlan.zhihu.com/p/34074528
    fn smooth_path(&self, astar: &mut AJPSStar<N>) {
        //去掉无用拐点
        let mut index1 = astar.result_indies.len() as i16 - 1;
        while index1 >= 2 {
            let mut index2 = 0;
            while index2 < index1 - 1 {
                if self.check_cross_walkable(
                    &self.nodes[astar.result_indies[index1 as usize]],
                    &self.nodes[astar.result_indies[index2 as usize]],
                ) {
                    let remove_index = index2 + 1;
                    while remove_index < index1 {
                        astar.result_indies.remove(remove_index as usize);
                        index1 -= 1;
                    }
                    break;
                }
                index2 += 1;
            }
            index1 -= 1;
        }
    }

    // 获取from节点direction方向上的节点在地图集合中的索引
    fn get_node_direction(&self, from: usize, direction: Direction) -> Option<usize> {
        let from_node = &self.nodes[from];
        match direction {
            Direction::Up => {
                if from_node.row != self.max_row {
                    Some(from_node.index + self.max_column + 1)
                } else {
                    None
                }
            }
            Direction::UpRight => {
                if from_node.row != self.max_row && from_node.column != self.max_column {
                    Some(from_node.index + self.max_column + 2)
                } else {
                    None
                }
            }
            Direction::Right => {
                if from_node.column != self.max_column {
                    Some(from_node.index + 1)
                } else {
                    None
                }
            }
            Direction::DownRight => {
                if from_node.row != 0 && from_node.column != self.max_column {
                    Some(from_node.index - self.max_column)
                } else {
                    None
                }
            }
            Direction::Down => {
                if from_node.row != 0 {
                    Some(from_node.index - self.max_column - 1)
                } else {
                    None
                }
            }
            Direction::DownLeft => {
                if from_node.column != 0 && from_node.row != 0 {
                    Some(from_node.index - self.max_column - 2)
                } else {
                    None
                }
            }
            Direction::Left => {
                if from_node.column != 0 {
                    Some(from_node.index - 1)
                } else {
                    None
                }
            }
            Direction::UpLeft => {
                if from_node.column != 0 && from_node.row != self.max_row {
                    Some(from_node.index + self.max_column)
                } else {
                    None
                }
            }
        }
    }

    //判断路径上是否有障碍物或代价缩放大的点
    fn check_cross_walkable(&self, p1: &TestJPSNode, p2: &TestJPSNode) -> bool {
        let half: N = FromPrimitive::from_f32(0.5).unwrap();
        let quarter: N = FromPrimitive::from_f32(0.25).unwrap();
        let temp_1: N = FromPrimitive::from_usize(p1.column).unwrap();
        let temp_2: N = FromPrimitive::from_usize(p1.row).unwrap();
        let point_1 = Point2::new(
            (temp_1 + half) * self.cell_size + self.start_pos.x,
            (temp_2 + half) * self.cell_size + self.start_pos.y,
        );

        let temp_1: N = FromPrimitive::from_usize(p2.column).unwrap();
        let temp_2: N = FromPrimitive::from_usize(p2.row).unwrap();
        let point_2 = Point2::new(
            (temp_1 + half) * self.cell_size + self.start_pos.x,
            (temp_2 + half) * self.cell_size + self.start_pos.y,
        );

        let dist_x = (p2.column as i16 - p1.column as i16).abs();
        let dist_y = (p2.row as i16 - p1.row as i16).abs();

        //根据起点重点间纵横向局里的大小来判断遍历方向
        if dist_x > dist_y {
            let min_x_pos: Point2<N>;
            let max_x_pos: Point2<N>;
            if point_1.x <= point_2.x {
                min_x_pos = point_1;
                max_x_pos = point_2;
            } else {
                min_x_pos = point_2;
                max_x_pos = point_1;
            }
            let mut cur_x = min_x_pos.x + quarter;
            //开始横向遍历起点与终点间的节点看是否存在障碍
            while cur_x < max_x_pos.x {
                let slop = (max_x_pos.y - min_x_pos.y) / (max_x_pos.x - min_x_pos.x);
                let cur_y = slop * cur_x + (max_x_pos.y - slop * max_x_pos.x);
                let cur_index = self.get_node_by_position(Point2::new(cur_x, cur_y));
                if self.nodes[cur_index].is_block {
                    return false;
                }
                cur_x = cur_x + quarter;
            }
        } else {
            let min_y_pos: &Point2<N>;
            let max_y_pos: &Point2<N>;
            if point_1.y <= point_2.y {
                min_y_pos = &point_1;
                max_y_pos = &point_2;
            } else {
                min_y_pos = &point_2;
                max_y_pos = &point_1;
            }
            let mut cur_y = min_y_pos.y + quarter;
            //开始横向遍历起点与终点间的节点看是否存在障碍
            while cur_y < max_y_pos.y {
                let mut cur_x = min_y_pos.x;
                if max_y_pos.x != min_y_pos.x {
                    let slop = (max_y_pos.y - min_y_pos.y) / (max_y_pos.x - min_y_pos.x);
                    cur_x = (cur_y - (max_y_pos.y - slop * max_y_pos.x)) / slop;
                }
                let cur_index = self.get_node_by_position(Point2::new(cur_x, cur_y));
                if self.nodes[cur_index].is_block {
                    return false;
                }
                cur_y = quarter + cur_y;
            }
        }
        return true;
    }

    // 获取from节点的direction方向上的跳点在地图集合中的索引
    fn get_jump_point_direction(
        &self,
        from: usize,
        direction: Direction,
        root: usize,
    ) -> Option<usize> {
        match self.get_node_direction(from, direction) {
            None => {
                return None;
            }
            Some(node_index) => {
                if self.nodes[node_index].is_block {
                    if from == root {
                        return None;
                    } else {
                        return Some(from);
                    }
                }
                if self.target_row * (self.max_column + 1) + self.target_column == node_index {
                    return Some(node_index);
                }
                if self.is_jump_point(node_index, direction) {
                    return Some(node_index);
                }
                if (self.nodes[node_index].row == self.target_row
                    || self.nodes[node_index].column == self.target_column)
                    && self.check_cross_walkable(
                        &self.nodes[node_index],
                        &self.nodes[self.target_row * (self.max_column + 1) + self.target_column],
                    )
                {
                    if !((self.nodes[root].row == self.target_row
                        || self.nodes[root].column == self.target_column)
                        && self.check_cross_walkable(
                            &self.nodes[root],
                            &self.nodes
                                [self.target_row * (self.max_column + 1) + self.target_column],
                        ))
                    {
                        return Some(node_index);
                    }
                }

                return self.get_jump_point_direction(node_index, direction, root);
            }
        }
    }

    //判定node_idx节点在move_direction方向上是否满足跳点的规则
    fn is_jump_point(&self, node_idx: usize, move_direction: Direction) -> bool {
        match move_direction {
            Direction::Up | Direction::Down | Direction::Left | Direction::Right => {
                let left_down = Direction::from_isize(move_direction as isize - 3);
                let left_down_idx = self.get_node_direction(node_idx, left_down);
                let left = Direction::from_isize(move_direction as isize - 2);
                let left_idx = self.get_node_direction(node_idx, left);
                if left_down_idx != None
                    && left_idx != None
                    && self.nodes[left_down_idx.unwrap()].is_block
                    && !self.nodes[left_idx.unwrap()].is_block
                {
                    return true;
                }
                let right_down = Direction::from_isize(move_direction as isize + 3);
                let right = Direction::from_isize(move_direction as isize + 2);
                let right_down_idx = self.get_node_direction(node_idx, right_down);
                let right_idx = self.get_node_direction(node_idx, right);
                if right_down_idx != None
                    && right_idx != None
                    && self.nodes[right_down_idx.unwrap()].is_block
                    && !self.nodes[right_idx.unwrap()].is_block
                {
                    return true;
                }
                return false;
            }
            _ => {
                return false;
            }
        }
    }

    // 获取to节点在from节点的哪个方向上
    fn get_direction(&self, from: usize, to: usize) -> Direction {
        if self.nodes[to].column == self.nodes[from].column
            && self.nodes[to].row > self.nodes[from].row
        {
            return Direction::Up;
        } else if self.nodes[to].column > self.nodes[from].column
            && self.nodes[to].row > self.nodes[from].row
        {
            return Direction::UpRight;
        } else if self.nodes[to].column > self.nodes[from].column
            && self.nodes[to].row == self.nodes[from].row
        {
            return Direction::Right;
        } else if self.nodes[to].column > self.nodes[from].column
            && self.nodes[to].row < self.nodes[from].row
        {
            return Direction::DownRight;
        } else if self.nodes[to].column == self.nodes[from].column
            && self.nodes[to].row < self.nodes[from].row
        {
            return Direction::Down;
        } else if self.nodes[to].column < self.nodes[from].column
            && self.nodes[to].row < self.nodes[from].row
        {
            return Direction::DownLeft;
        } else if self.nodes[to].column < self.nodes[from].column
            && self.nodes[to].row == self.nodes[from].row
        {
            return Direction::Left;
        } else {
            return Direction::UpLeft;
        }
    }
}

impl Iterator for ATestJPSNodeIterator {
    type Item = usize;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cur_index >= self.neighbors.len() {
            return None;
        }
        let result = self.neighbors[self.cur_index];
        self.cur_index += 1;
        Some(result)
    }
}

// JPS方格节点
struct TestJPSNode {
    row: usize,
    column: usize,
    index: usize,
    //是否为阻挡
    is_block: bool,
    //是否为目标点
    is_target: bool,
}

impl<'a, N> AJPSStarMap<'a> for TestJPSMap<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    type AStartNodeIter = ATestJPSNodeIterator;

    // 获取遍历邻居节点（跳点）的迭代器
    fn get_neighbors(&self, cur: usize, parent: Option<usize>) -> ATestJPSNodeIterator {
        let mut neighbors = Vec::with_capacity(8);
        match parent {
            None => {
                for dir in 0..8 {
                    let jump = self.get_jump_point_direction(
                        cur,
                        FromPrimitive::from_usize(dir).unwrap(),
                        cur,
                    );
                    if jump != None {
                        neighbors.push(jump.unwrap())
                    }
                }
            }
            Some(parent_idx) => {
                let parent_dir = self.get_direction(parent_idx, cur) as isize;

                let jump =
                    self.get_jump_point_direction(cur, Direction::from_isize(parent_dir), cur);
                if jump != None {
                    neighbors.push(jump.unwrap())
                }

                let jump =
                    self.get_jump_point_direction(cur, Direction::from_isize(parent_dir - 1), cur);
                if jump != None {
                    neighbors.push(jump.unwrap())
                }

                let jump =
                    self.get_jump_point_direction(cur, Direction::from_isize(parent_dir + 1), cur);
                if jump != None {
                    neighbors.push(jump.unwrap())
                }

                let jump =
                    self.get_jump_point_direction(cur, Direction::from_isize(parent_dir - 2), cur);
                if jump != None {
                    neighbors.push(jump.unwrap())
                }

                let jump =
                    self.get_jump_point_direction(cur, Direction::from_isize(parent_dir + 2), cur);
                if jump != None {
                    neighbors.push(jump.unwrap())
                }
            }
        }

        ATestJPSNodeIterator {
            cur_index: 0,
            neighbors: neighbors,
        }
    }
}

// 八方向枚举
#[derive(Clone, Copy, FromPrimitive, PartialEq)]
enum Direction {
    Up = 0,
    UpRight = 1,
    Right = 2,
    DownRight = 3,
    Down = 4,
    DownLeft = 5,
    Left = 6,
    UpLeft = 7,
}

impl Direction {
    #[inline]
    fn from_isize(v: isize) -> Direction {
        let mut v = v;
        v += 8;
        v = v % 8;
        FromPrimitive::from_isize(v).unwrap()
    }
}

// ============================= 测试用例

#[cfg(test)]
mod test_jpsfindpath {
    use nalgebra::Point2;
    use raqote::*;
    #[test]
    fn it_works() {
        let cell_size = 1.0;

        let start_pos: Point2<f32> = Point2::new(0.0, 0.0);
        let mut map = crate::test_jps::TestJPSMap::new_map(100, 100, cell_size, start_pos, 5);
        let contents = std::fs::read_to_string("SampleScene.txt")
            .expect("Something went wrong reading the file");
        for (row, line) in contents.lines().enumerate() {
            for (column, contant) in line.split(" ").enumerate() {
                if contant == "1" {
                    map.set_block(
                        Point2::new(column as f32, row as f32),
                        Point2::new(column as f32 + 1.0, row as f32 + 1.0),
                        true,
                    )
                }
            }
        }

        let path = map.get_path(Point2::new(88.6, 4.4), Point2::new(6.8, 74.6), 99999);
        let draw_cell_size = 10;

        let mut dt = DrawTarget::new(
            (map.max_column as i32 + 1) * draw_cell_size,
            (map.max_row as i32 + 1) * draw_cell_size,
        );
        let draw_opt = DrawOptions::new();
        let block_source = Source::Solid(SolidSource::from(Color::new(255, 0, 0, 0)));
        for node in map.nodes {
            if node.is_block {
                dt.fill_rect(
                    node.column as f32 * draw_cell_size as f32,
                    node.row as f32 * draw_cell_size as f32,
                    draw_cell_size as f32,
                    draw_cell_size as f32,
                    &block_source,
                    &draw_opt,
                );
            }
        }
        let path_source = Source::Solid(SolidSource::from(Color::new(255, 0, 250, 0)));
        for path_point in path {
            // print!("{},{}->", path_point.x , path_point.y);
            let start_x = (path_point.x - cell_size / 2.0) as f32 * draw_cell_size as f32;
            let start_y = (path_point.y - cell_size / 2.0) as f32 * draw_cell_size as f32;
            dt.fill_rect(
                start_x,
                start_y,
                draw_cell_size as f32,
                draw_cell_size as f32,
                &path_source,
                &draw_opt,
            );
        }
        let line_source = Source::Solid(SolidSource::from(Color::new(255, 0, 0, 0)));
        let mut path_builder = PathBuilder::new();
        let stroke_style = StrokeStyle {
            cap: LineCap::Round,
            join: LineJoin::Round,
            width: 2.,
            miter_limit: 2.,
            dash_array: vec![10., 0.],
            dash_offset: 16.,
        };
        let mut index = 0;
        while index < map.max_column {
            path_builder.move_to((index as f32 + 1.0) * draw_cell_size as f32, 0.0);
            path_builder.line_to(
                (index as f32 + 1.0) * draw_cell_size as f32,
                dt.height() as f32,
            );
            index += 1;
        }
        index = 0;
        while index < map.max_row {
            path_builder.move_to(0.0, (index as f32 + 1.0) * draw_cell_size as f32);
            path_builder.line_to(
                dt.width() as f32,
                (index as f32 + 1.0) * draw_cell_size as f32,
            );
            index += 1;
        }
        let draw_path = path_builder.finish();
        dt.stroke(&draw_path, &line_source, &stroke_style, &draw_opt);
        dt.write_png("test_jps_path.png");
    }
}

#[cfg(test)]
mod test_jpstests {
    use nalgebra::Point2;
    use test::Bencher;

    #[bench]
    fn bench_test(b: &mut Bencher) {
        let cell_size = 1.0;

        let start_pos: Point2<f32> = Point2::new(0.0, 0.0);
        let mut map = crate::test_jps::TestJPSMap::new_map(100, 100, cell_size, start_pos, 5);
        let contents = std::fs::read_to_string("SampleScene.txt")
            .expect("Something went wrong reading the file");
        for (row, line) in contents.lines().enumerate() {
            for (column, contant) in line.split(" ").enumerate() {
                if contant == "1" {
                    map.set_block(
                        Point2::new(column as f32, row as f32),
                        Point2::new(column as f32 + 1.0, row as f32 + 1.0),
                        true,
                    )
                }
            }
        }

        b.iter(|| map.get_path(Point2::new(88.6, 4.4), Point2::new(6.8, 74.6), 99999));
    }
}
