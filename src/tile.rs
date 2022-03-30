//!
//! 基于网格图的A*寻路
//!

use crate::{AFilter, AStar, AStarMap};
use nalgebra::{Point2, Scalar};
use num_traits::{cast::AsPrimitive, FromPrimitive, Num};
use std::marker::PhantomData;

/// ## 方格A*寻路的寻路主体代理约束
///
/// +通过代理将不同寻路能力主体的寻路情况交由上层逻辑决定，增加导航图的通用性增加导航图的通用性及寻路能力的动态性
///
/// 需要实现主体在指定点是否被阻挡
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
///
/// 示例：
/// # Examples
/// ```
/// struct TestFilter {}
/// impl ATileFilter<f32> for TestFilter {
///     fn is_block(&self, pos_x: N, pos_y: N) -> bool {
///         if pos_x >= 7.0 && pos_x <= 8.0 && pos_y >= 0.0 && pos_y <= 9.0 {
///             return true;
///         } else if pos_x >= 8.0 && pos_x <= 16.0 && pos_y >= 8.0 && pos_y <= 9.0 {
///             return true;
///         }
///         false
///     }
/// }
/// ```
pub trait ATileFilter<N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    /// 传入点是否为阻挡
    fn is_block(&self, pos_x: N, pos_y: N) -> bool;
}

/// ## 方格A* 寻路的方格图
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
///
pub struct TileMap<'a, N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    // 该图所有节点
    nodes: Vec<TileNode<'a>>,
    // 该图最大行数
    max_row: usize,
    // 该图最大列数
    max_column: usize,
    // 该图节点的尺寸
    cell_size: N,
    // 该图起点位于场景中的坐标
    start_pos: Point2<N>,
    // 调用A*的内部存储结构,支持多线程（一张图同时多个寻路请求）
    astar: Vec<AStar<N>>,
}

impl<'a, N> TileMap<'a, N>
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
    ) -> TileMap<'a, N> {
        let mut map = TileMap {
            nodes: Vec::with_capacity(row * column),
            max_row: row - 1,
            max_column: column - 1,
            cell_size: cell_size,
            start_pos: start_pos,
            astar: Vec::with_capacity(max_find_num_in_time),
        };
        for _ in 0..max_find_num_in_time {
            map.astar.push(AStar::new(row * column));
        }

        let mut cur_row = 0;
        while cur_row < row {
            let mut cur_column = 0;
            while cur_column < column {
                let mut node = TileNode {
                    row: cur_row,
                    column: cur_column,
                    index: cur_row * (map.max_column + 1) + cur_column,
                    map_max_row: map.max_row,
                    map_max_column: map.max_column,
                    neighbors: Vec::with_capacity(8),
                    phantom: PhantomData::default(),
                };
                node.init_neighbors();
                map.nodes.push(node);
                cur_column = cur_column + 1;
            }
            cur_row = cur_row + 1;
        }
        map
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
    pub fn get_path<Filter: ATileFilter<N>>(
        &mut self,
        start_pos: Point2<N>,
        end_pos: Point2<N>,
        filter: &Filter,
        max_nodes: usize,
    ) -> Vec<Point2<N>> {
        let start_node_index = self.get_node_by_position(start_pos);
        let end_node_index = self.get_node_by_position(end_pos);

        let tile_filter = TileFilter {
            filter: filter,
            map_max_column: self.max_column,
            map_cell_size: self.cell_size,
            map_start_pos: self.start_pos,
        };
        let mut astar = if self.astar.len() > 0 {
            self.astar.pop().unwrap()
        } else {
            AStar::new(self.nodes.len())
        };
        astar.find_path(
            self,
            start_node_index,
            end_node_index,
            &tile_filter,
            max_nodes,
        );
        self.smooth_path(&mut astar, &tile_filter);
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

// 遍历邻居的迭代器
pub struct ATileNodeIterator<'a> {
    neighbors_ref: &'a Vec<usize>,
    cur_index: usize,
}

//============================= 本地
//对astar的寻路主体代理的封装
struct TileFilter<'a, N, Filter>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
    Filter: ATileFilter<N>,
{
    filter: &'a Filter,
    map_cell_size: N,
    map_start_pos: Point2<N>,
    map_max_column: usize,
}

impl<'a, N, Filter> AFilter<N> for TileFilter<'a, N, Filter>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
    Filter: ATileFilter<N>,
{
    // 从`from`节点到`to`节点是否可达
    fn is_pass(&self, _: usize, to: usize) -> bool {
        let to_column = to % (self.map_max_column + 1);
        let to_row = to / (self.map_max_column + 1);
        let x: N = FromPrimitive::from_usize(to_column).unwrap();
        let y: N = FromPrimitive::from_usize(to_row).unwrap();
        let half: N = FromPrimitive::from_f32(0.5).unwrap();
        !self.filter.is_block(
            (x + half) * self.map_cell_size + self.map_start_pos.x,
            (y + half) * self.map_cell_size + self.map_start_pos.y,
        )
    }

    // 从当前`cur`节点到`end`节点的`h`，即从当前节点到终点的预估代价
    fn get_h(&self, cur: usize, end: usize) -> N {
        let to_column = end % (self.map_max_column + 1);
        let to_row = end / (self.map_max_column + 1);
        let from_column = cur % (self.map_max_column + 1);
        let from_row = cur / (self.map_max_column + 1);
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

    // 从`parent`节点到当前`cur`节点的`g`，即从父节点到当前节点的实际代价
    fn get_g(&self, cur: usize, parent: usize) -> N {
        let to_column = cur % (self.map_max_column + 1);
        let to_row = cur / (self.map_max_column + 1);
        let from_column = parent % (self.map_max_column + 1);
        let from_row = parent / (self.map_max_column + 1);
        if from_row == to_row || from_column == to_column {
            N::one()
        } else {
            FromPrimitive::from_f32(1.4142135).unwrap() //根号2
        }
    }
}

impl<'a, N> TileMap<'a, N>
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
    fn smooth_path<Filter: AFilter<N>>(&self, astar: &mut AStar<N>, filter: &Filter) {
        //删除同一路径上的节点
        if astar.result_indies.len() > 2 {
            let mut vector_x = self.nodes[astar.result_indies[astar.result_indies.len() - 1]].column
                as i16
                - self.nodes[astar.result_indies[astar.result_indies.len() - 2]].column as i16;
            let mut vector_y = self.nodes[astar.result_indies[astar.result_indies.len() - 1]].row
                as i16
                - self.nodes[astar.result_indies[astar.result_indies.len() - 2]].row as i16;

            let mut index = astar.result_indies.len() as i16 - 3;
            while index >= 0 {
                let temp_vector_x = self.nodes[astar.result_indies[index as usize + 1]].column
                    as i16
                    - self.nodes[astar.result_indies[index as usize]].column as i16;
                let temp_vector_y = self.nodes[astar.result_indies[index as usize + 1]].row as i16
                    - self.nodes[astar.result_indies[index as usize]].row as i16;

                if vector_x == temp_vector_x && vector_y == temp_vector_y {
                    astar.result_indies.remove(index as usize + 1);
                } else {
                    vector_x = temp_vector_x;
                    vector_y = temp_vector_y;
                }
                index -= 1;
            }
        }

        //去掉无用拐点
        let mut index1 = astar.result_indies.len() as i16 - 1;
        while index1 >= 2 {
            let mut index2 = 0;
            while index2 < index1 - 1 {
                if self.check_cross_walkable(
                    &self.nodes[astar.result_indies[index1 as usize]],
                    &self.nodes[astar.result_indies[index2 as usize]],
                    filter,
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

    //判断路径上是否有障碍物或代价缩放大的点
    fn check_cross_walkable<Filter: AFilter<N>>(
        &self,
        p1: &TileNode,
        p2: &TileNode,
        filter: &Filter,
    ) -> bool {
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
                if !filter.is_pass(cur_index, cur_index) {
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
                if !filter.is_pass(cur_index, cur_index) {
                    return false;
                }
                cur_y = quarter + cur_y;
            }
        }
        return true;
    }
}

impl<'a> Iterator for ATileNodeIterator<'a> {
    type Item = usize;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cur_index >= self.neighbors_ref.len() {
            return None;
        }
        let result = self.neighbors_ref[self.cur_index];
        self.cur_index += 1;
        Some(result)
    }
}

// 方格节点
struct TileNode<'a> {
    row: usize,
    column: usize,
    index: usize,
    map_max_row: usize,
    map_max_column: usize,
    neighbors: Vec<usize>,

    phantom: PhantomData<&'a ()>,
}

impl<'a> TileNode<'a> {
    //初始化方格节点的所有邻居节点
    fn init_neighbors(&mut self) {
        if self.row != self.map_max_row {
            self.neighbors.push(self.index + self.map_max_column + 1);
        }
        if self.column != 0 {
            self.neighbors.push(self.index - 1);
        }
        if self.column != self.map_max_column {
            self.neighbors.push(self.index + 1);
        }
        if self.row != 0 {
            self.neighbors.push(self.index - self.map_max_column - 1);
        }
        if self.column != 0 && self.row != self.map_max_row {
            self.neighbors.push(self.index + self.map_max_column);
        }
        if self.row != self.map_max_row && self.column != self.map_max_column {
            self.neighbors.push(self.index + self.map_max_column + 2);
        }
        if self.column != 0 && self.row != 0 {
            self.neighbors.push(self.index - self.map_max_column - 2);
        }
        if self.row != 0 && self.column != self.map_max_column {
            self.neighbors.push(self.index - self.map_max_column);
        }
    }
}

impl<'a, N> AStarMap<'a> for TileMap<'a, N>
where
    N: Scalar + Num + AsPrimitive<usize> + FromPrimitive + PartialOrd,
{
    type AStartNodeIter = ATileNodeIterator<'a>;

    // 获取遍历邻居节点的迭代器
    fn get_neighbors(&'a self, cur: usize, _: Option<usize>) -> ATileNodeIterator {
        ATileNodeIterator {
            neighbors_ref: &self.nodes[cur].neighbors,
            cur_index: 0,
        }
    }
}

// ============================= 测试用例

#[cfg(test)]
mod tileastar {
    use crate::ATileFilter;
    use nalgebra::Point2;
    use raqote::*;
    #[test]
    fn it_works() {
        let cell_size = 1.0;
        let start_pos: Point2<f32> = Point2::new(0.0, 0.0);
        let mut map = crate::tile::TileMap::new_map(100, 100, cell_size, start_pos, 5);

        struct TestFilter {
            max_column: usize,
            is_blocks: Vec<bool>,
        }

        impl ATileFilter<f32> for TestFilter {
            fn is_block(&self, pos_x: f32, pos_y: f32) -> bool {
                let column = pos_x as usize;
                let row = pos_y as usize;

                return self.is_blocks[row * (self.max_column + 1) + column];
            }
        }

        let mut filter = TestFilter {
            is_blocks: Vec::new(),
            max_column: map.max_column,
        };
        let contents = std::fs::read_to_string("SampleScene.txt")
            .expect("Something went wrong reading the file");
        for (_, line) in contents.lines().enumerate() {
            for (_, contant) in line.split(" ").enumerate() {
                if contant == "1" {
                    filter.is_blocks.push(true);
                } else if contant == "0" {
                    filter.is_blocks.push(false);
                }
            }
        }

        let path = map.get_path(
            Point2::new(88.6, 4.4),
            Point2::new(6.8, 74.6),
            &filter,
            99999,
        );
        let draw_cell_size = 10;

        let mut dt = DrawTarget::new(
            (map.max_column as i32 + 1) * draw_cell_size,
            (map.max_row as i32 + 1) * draw_cell_size,
        );
        let draw_opt = DrawOptions::new();
        let block_source = Source::Solid(SolidSource::from(Color::new(255, 0, 0, 0)));
        for node in map.nodes {
            let x = node.column as f32;
            let y = node.row as f32;
            if filter.is_block(x + 0.5, y + 0.5) {
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
        dt.write_png("tile_path.png");
    }
}

#[cfg(test)]
mod tiletests {
    use crate::ATileFilter;
    use nalgebra::Point2;
    use test::Bencher;

    #[bench]
    fn bench_test(b: &mut Bencher) {
        let cell_size = 1.0;
        let start_pos: Point2<f32> = Point2::new(0.0, 0.0);
        let mut map = crate::tile::TileMap::new_map(100, 100, cell_size, start_pos, 5);

        struct TestFilter {
            max_column: usize,
            is_blocks: Vec<bool>,
        }

        impl ATileFilter<f32> for TestFilter {
            fn is_block(&self, pos_x: f32, pos_y: f32) -> bool {
                let column = pos_x as usize;
                let row = pos_y as usize;

                return self.is_blocks[row * (self.max_column + 1) + column];
            }
        }

        let mut filter = TestFilter {
            is_blocks: Vec::new(),
            max_column: map.max_column,
        };
        let contents = std::fs::read_to_string("SampleScene.txt")
            .expect("Something went wrong reading the file");
        for (_, line) in contents.lines().enumerate() {
            for (_, contant) in line.split(" ").enumerate() {
                if contant == "1" {
                    filter.is_blocks.push(true);
                } else if contant == "0" {
                    filter.is_blocks.push(false);
                }
            }
        }

        b.iter(|| {
            map.get_path(
                Point2::new(88.6, 4.4),
                Point2::new(6.8, 74.6),
                &filter,
                99999,
            )
        });
    }
}
