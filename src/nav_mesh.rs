//!
//! 导航网格 的 A* 寻路
//!

use crate::{make_neighbors, AStar, AStarResult, Entry, Map, NodeIndex};
use nalgebra::{Point3, RealField, Vector3};
use std::{collections::HashMap, fmt::Debug};

// 在 某个 多边形 点列表 的 索引
#[derive(Debug, Default, Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct PointIndex(usize);

impl From<usize> for PointIndex {
    fn from(v: usize) -> Self {
        Self(v)
    }
}

impl From<PointIndex> for usize {
    fn from(v: PointIndex) -> Self {
        v.0
    }
}

/// ## 导航网格 的 A* 寻路
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
///
#[derive(Debug)]
pub struct NavMeshMap<N: Copy + RealField + Debug> {
    // 该导航网格 所有的 顶点
    points: Vec<Point3<N>>,

    // 该导航网格 所有的边
    segments: Vec<Segment<N>>,

    // 凸多边形，其中的点，边 都是 索引
    polygons: Vec<Polygon<N>>,
}

impl<N: Copy + RealField + Debug> NavMeshMap<N> {
    ///
    /// 新建 导航网格
    ///
    /// `vertexs`传入构成 Nav-Mesh 所有顶点坐标 (x, y, z)
    ///
    /// `indexs` 每个元素 都是 一个凸多边形网格 的 顶点索引数组
    ///
    pub fn new(points: Vec<Point3<N>>, nav_indexs: &[Vec<usize>]) -> Self {
        let mut map = Self {
            points,
            segments: vec![],
            polygons: Vec::with_capacity(nav_indexs.len()),
        };

        // (索引小的顶点，索引大的顶点) -> SegmentIndex
        let mut segment_map: HashMap<(PointIndex, PointIndex), SegmentIndex> = HashMap::new();

        for indexs in nav_indexs.iter() {
            // 多边形
            let polygon_index = map.polygons.len().into();
            let p = Polygon::new(&mut map, &mut segment_map, polygon_index, indexs);
            map.polygons.push(p);
        }

        println!("seg 114 = {:?}", map.segments[114]);
        println!("seg 112 = {:?}", map.segments[112]);
        println!("seg 113 = {:?}", map.segments[113]);
        println!("seg 115 = {:?}", map.segments[115]);
        println!("seg 116 = {:?}", map.segments[116]);
        println!("seg 117 = {:?}", map.segments[117]);
        println!("seg 118 = {:?}", map.segments[118]);

        map
    }

    ///
    /// 通过A*算法获取寻路路径
    ///
    /// 需要指定场景中的开始坐标及目标坐标，寻路主体代理实现，以及最多访问Nav-Mesh线段的数量,
    ///
    /// is_simply_path 是否 简化路径（拉直）
    ///
    /// 返回 有效路径坐标序列
    ///
    /// + 超出Nav-Mesh图范围的开始点及目标点会被强行拉至Nav-Mesh图范围内
    fn find_path(
        &mut self,
        start_pos: Point3<N>,
        end_pos: Point3<N>,
        is_simply_path: bool,
        max_nodes: usize,
        max_numbers: usize,
    ) -> Result<Vec<Point3<N>>, String> {
        // 起点多边形
        let start_polygon = self.get_polygon_by_point(&start_pos);
        // 起点边
        let start_segment_index =
            self.polygons[start_polygon.0].get_nearst_segment_index(&start_pos, self);

        // 终点多边形
        let end_polygon = self.get_polygon_by_point(&end_pos);
        // 终点边
        let end_segment_index =
            self.polygons[end_polygon.0].get_nearst_segment_index(&end_pos, self);

        let mut astar: AStar<N, Entry<N>> = AStar::with_capacity(self.segments.len(), max_nodes);

        let start = NodeIndex(start_segment_index.0);
        let end = NodeIndex(end_segment_index.0);

        let r = astar.find(start, end, max_numbers, &mut |cur, end, finder| {
            let r = make_neighbors(self, cur, end, finder);
            println!("======== {:?}", r);
            r
        });

        match r {
            AStarResult::Found => {}
            AStarResult::NotFound => {
                return Err("AStarResult::NotFound".to_string());
            }
            AStarResult::LimitNotFound(_) => {
                return Err("AStarResult::LimitNotFound".to_string());
            }
        }

        let mut result_indies: Vec<usize> = astar.result_iter(end).map(|v| v.0).collect();
        result_indies.reverse();

        if self.polygons[start_polygon.0].is_contain_segment(SegmentIndex(result_indies[0])) {
            // 由于找最近边为起点边，产生了折现（）路径第一个点和第二个点和起点在同一个多边形内）
            result_indies.remove(0);
        }

        let end_segment_index = result_indies[result_indies.len() - 1];
        if self.polygons[end_polygon.0].is_contain_segment(SegmentIndex(end_segment_index)) {
            result_indies.pop();
        }

        let mut pts = if is_simply_path {
            self.funnel(&mut result_indies, start_pos)
        } else {
            self.get_path_points(&result_indies, start_pos)
        };
        pts.push(end_pos);

        Ok(pts)
    }

    fn get_path_points(&self, result_indies: &[usize], start: Point3<N>) -> Vec<Point3<N>> {
        let mut r = vec![start];
        for i in result_indies {
            r.push(self.segments[*i].center);
        }
        r
    }
}

impl<N: Copy + RealField + Debug> Map<N> for NavMeshMap<N> {
    type NodeIter = NavMeshNodeIterator;

    // 注：这里的 NodeIndex 是 SegmentIndex
    fn get_neighbors(&self, cur: NodeIndex, parent: NodeIndex) -> Self::NodeIter {
        let seg = &self.segments[cur.0];
        NavMeshNodeIterator::new(seg.neighbors.clone(), parent.0.into())
    }

    // 注：这里的 NodeIndex 是 SegmentIndex
    // 用 边的 中心 来计算 距离
    fn get_g(&self, cur: NodeIndex, parent: NodeIndex) -> N {
        let cur = &self.segments[cur.0].center;
        let parent = &self.segments[parent.0].center;
        nalgebra::distance(parent, cur)
    }

    // 注：这里的 NodeIndex 是 SegmentIndex
    // 用 边的 中心 来计算 距离
    fn get_h(&self, cur: NodeIndex, end: NodeIndex) -> N {
        let cur = &self.segments[cur.0].center;
        let parent = &self.segments[end.0].center;

        nalgebra::distance(parent, cur)
    }
}

//============================= 本地

// 遍历邻居的迭代器
pub struct NavMeshNodeIterator {
    // 当前遍历的 迭代器索引
    index: usize,

    exclusive: SegmentIndex,

    neighbors: Vec<SegmentIndex>,
}

impl NavMeshNodeIterator {
    fn new(neighbors: Vec<SegmentIndex>, exclusive: SegmentIndex) -> Self {
        let index = neighbors.len();
        Self {
            neighbors,
            exclusive,
            index,
        }
    }
}

impl Iterator for NavMeshNodeIterator {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index == 0 {
            return None;
        }

        self.index -= 1;

        let seg = self.neighbors[self.index];
        if self.exclusive != seg {
            Some(NodeIndex(seg.0))
        } else {
            self.next()
        }
    }
}

impl<N: Copy + RealField + Debug> NavMeshMap<N> {
    // 获取给定点所在的多边形，若该点不处于任何多边形内，则查找最近多边形
    fn get_polygon_by_point(&self, point: &Point3<N>) -> PolygonIndex {
        for polygon_ref in self.polygons.iter() {
            if polygon_ref.is_contain_point(&self.points, point) {
                return polygon_ref.index;
            }
        }

        // 点不在任何多边形内
        let mut nearst_index: PolygonIndex = Default::default();
        let mut nearst_distance_sqr =
            nalgebra::distance_squared(&self.polygons[nearst_index.0].center, point);

        for polygon in &self.polygons {
            let distance_sqr = nalgebra::distance_squared(&polygon.center, point);
            if distance_sqr < nearst_distance_sqr {
                nearst_index = polygon.index;
                nearst_distance_sqr = distance_sqr;
            }
        }

        self.polygons[nearst_index.0].index
    }

    // 检查三个向量是否按顺序排布
    fn vertor_is_order(v1: &Vector3<N>, v2: &Vector3<N>, v3: &Vector3<N>) -> bool {
        let cross1 = v2.cross(v1);
        let cross2 = v2.cross(v3);

        cross1.dot(&cross2) <= N::zero()
    }

    // 拉直: 漏斗算法
    // http://liweizhaolili.lofter.com/post/1cc70144_86a939e
    // http://digestingduck.blogspot.hk/2010/03/simple-stupid-funnel-algorithm.html
    fn funnel(&self, segments_indies: &mut [usize], start: Point3<N>) -> Vec<Point3<N>> {
        let mut points: Vec<Point3<N>> = Vec::new();

        if segments_indies.len() == 1 {
            return points;
        }

        let seg = &self.segments[segments_indies[0]];

        let mut old_v1 = self.points[seg.start.0] - start;
        let mut old_v2 = self.points[seg.end.0] - start;

        let mut last_point = start;

        points.push(start);

        let mut index = 1;
        while index < segments_indies.len() {
            let seg = &self.segments[segments_indies[index]];

            let new_v1 = self.points[seg.start.0] - last_point;
            let new_v2 = self.points[seg.end.0] - last_point;

            let mut pos = None;
            if !NavMeshMap::vertor_is_order(&old_v1, &new_v1, &old_v2) {
                //new_v1在old_v1，old_v2范围外
                if NavMeshMap::vertor_is_order(&new_v1, &old_v1, &old_v2) {
                    // 靠近old_v1
                    pos = Some(last_point + old_v1);
                } else {
                    // 靠近old_v2
                    pos = Some(last_point + old_v2);
                }
            } else if !NavMeshMap::vertor_is_order(&old_v1, &new_v2, &old_v2) {
                // new_v2在old_v1，old_v2范围外
                if NavMeshMap::vertor_is_order(&new_v2, &old_v1, &old_v2) {
                    //靠近old_v1
                    pos = Some(last_point + old_v1);
                } else {
                    //靠近old_v2
                    pos = Some(last_point + old_v2);
                }
            }

            if pos != None {
                points.push(pos.unwrap());
                last_point = pos.unwrap();
            }

            let seg = &self.segments[segments_indies[index]];
            old_v1 = self.points[seg.start.0] - last_point;
            old_v2 = self.points[seg.end.0] - last_point;

            index += 1;
        }

        points
    }
}

// 多边形
#[derive(Debug)]
struct Polygon<N: Copy + RealField + Debug> {
    // 多边形数组 的 索引
    index: PolygonIndex,

    // 中心点
    center: Point3<N>,

    // 顶点集合
    points: Vec<PointIndex>,

    // 边集合
    segments: Vec<SegmentIndex>,

    // 包围盒
    aabb: AABB<N>,
}

impl<N: Copy + RealField + Debug> Polygon<N> {
    fn new(
        map: &mut NavMeshMap<N>,
        segment_map: &mut HashMap<(PointIndex, PointIndex), SegmentIndex>,
        index: PolygonIndex,
        indexs: &[usize],
    ) -> Self {
        let mut p = Self {
            index,
            center: Default::default(),
            points: vec![],
            segments: vec![],
            aabb: Default::default(),
        };

        // 点
        for i in indexs.iter() {
            p.add_point(map.points.as_slice(), (*i).into());
        }

        p.compute_center(map.points.as_slice());

        // 边
        for i in 0..indexs.len() {
            let j = (i + 1) % indexs.len();

            let mut begin = indexs[i].into();
            let mut end = indexs[j].into();
            if begin > end {
                std::mem::swap(&mut begin, &mut end);
            }

            let s_index = match segment_map.get(&(begin, end)) {
                Some(index) => *index,
                None => {
                    // 没有创建
                    let s = Segment::new(map.points.as_slice(), begin, end);

                    let s_index = SegmentIndex(map.segments.len());

                    map.segments.push(s);

                    segment_map.insert((begin, end), s_index);

                    s_index
                }
            };
            p.add_segment(s_index);
        }

        // 邻居
        for s in &p.segments {
            let segment = &mut map.segments[s.0];
            for n in &p.segments {
                if n.0 != s.0 {
                    segment.add_neighbor(*n);
                }
            }
        }

        p
    }

    /// 加 点
    fn add_point(&mut self, pts: &[Point3<N>], index: PointIndex) {
        let pt = &pts[index.0];

        self.aabb.add_point(pt);
        self.points.push(index);
    }

    // 计算 中心
    fn compute_center(&mut self, pts: &[Point3<N>]) {
        self.center = Default::default();

        for i in self.points.iter() {
            let pt = &pts[i.0];

            self.center.x += pt.x;
            self.center.y += pt.y;
            self.center.z += pt.z;
        }

        let point_num = N::from_usize(self.points.len()).unwrap();
        self.center.x /= point_num;
        self.center.y /= point_num;
        self.center.z /= point_num;
    }

    /// 加 边
    fn add_segment(&mut self, index: SegmentIndex) {
        self.segments.push(index);
    }
}

impl<N: Copy + RealField + Debug> Polygon<N> {
    fn is_contain_segment(&self, index: SegmentIndex) -> bool {
        self.segments.contains(&index)
    }

    // 给定点是否在多边形内
    fn is_contain_point(&self, points: &[Point3<N>], pt: &Point3<N>) -> bool {
        if !self.aabb.contain_point(pt) {
            return false;
        }

        let mut last_cross = None;
        for curr in 0..self.points.len() {
            let next = (curr + 1) % self.points.len();

            let pt_curr = &points[self.points[curr].0];
            let pt_next = &points[self.points[next].0];

            let v1 = pt_next - pt_curr;

            let v2 = pt - pt_curr;

            let cross = v1.cross(&v2);

            if last_cross != None && cross.dot(&last_cross.unwrap()) < N::zero() {
                return false;
            }
            last_cross = Some(cross);
        }

        true
    }

    // 给定点 到 该多边形 距离 最小的 边索引
    fn get_nearst_segment_index(&self, point: &Point3<N>, map: &NavMeshMap<N>) -> SegmentIndex {
        let mut nearst_segment_index: SegmentIndex = self.segments[0];

        let center0 = map.segments.get(nearst_segment_index.0).unwrap().center;

        let mut nearst_segment_distance_sqr = nalgebra::distance_squared(&center0, point);

        for segment_index in self.segments.iter() {
            let center = map.segments.get(segment_index.0).unwrap().center;
            let distance_sqr = nalgebra::distance_squared(&center, point);

            if distance_sqr < nearst_segment_distance_sqr {
                nearst_segment_index = *segment_index;
                nearst_segment_distance_sqr = distance_sqr;
            }
        }
        nearst_segment_index
    }
}

// 多边形列表的索引
#[derive(Debug, Default, Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
struct PolygonIndex(usize);

impl From<usize> for PolygonIndex {
    fn from(v: usize) -> Self {
        Self(v)
    }
}

impl From<PolygonIndex> for usize {
    fn from(v: PolygonIndex) -> Self {
        v.0
    }
}

// 在 边列表 的 索引
#[derive(Debug, Default, Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
struct SegmentIndex(usize);

impl From<usize> for SegmentIndex {
    fn from(v: usize) -> Self {
        Self(v)
    }
}

impl From<SegmentIndex> for usize {
    fn from(v: SegmentIndex) -> Self {
        v.0
    }
}

// 线段--Astart的Node
#[derive(Debug)]
struct Segment<N: Copy + RealField + Debug> {
    start: PointIndex, // 端点1
    end: PointIndex,   // 端点2

    center: Point3<N>, // 中点

    neighbors: Vec<SegmentIndex>, // 相邻的边的index
}

impl<N: Copy + RealField + Debug> Segment<N> {
    // 创建 边
    // pts 顶点集合
    // start，end 边的端点 在pts 的 索引
    fn new(pts: &[Point3<N>], start: PointIndex, end: PointIndex) -> Self {
        Self {
            start,
            end,
            center: pts[start.0] + (pts[end.0] - pts[start.0]) / N::from_f64(2.0).unwrap(),
            neighbors: vec![],
        }
    }

    // 添加 改边的邻居，也就是 可达的边 在 多边形边数组 的 索引
    fn add_neighbor(&mut self, index: SegmentIndex) {
        self.neighbors.push(index);
    }
}

// 包围盒
#[derive(Debug)]
struct AABB<N: Copy + RealField + Debug> {
    min_pt: Point3<N>,
    max_pt: Point3<N>,
}

impl<N: Copy + RealField + Debug> Default for AABB<N> {
    fn default() -> Self {
        let max = N::max_value().unwrap();
        let min = N::min_value().unwrap();
        Self {
            min_pt: Point3::new(max, max, max),
            max_pt: Point3::new(min, min, min),
        }
    }
}

impl<N: Copy + RealField + Debug> AABB<N> {
    // 添加点，扩展 AABB
    #[inline]
    fn add_point(&mut self, pt: &Point3<N>) {
        self.min_pt.x = self.min_pt.x.min(pt.x);
        self.min_pt.y = self.min_pt.y.min(pt.y);
        self.min_pt.y = self.min_pt.z.min(pt.z);

        self.max_pt.x = self.max_pt.x.max(pt.x);
        self.max_pt.y = self.max_pt.y.max(pt.y);
        self.max_pt.y = self.max_pt.z.max(pt.z);
    }

    // 给定点是否在 AABB 内
    #[inline]
    fn contain_point(&self, pt: &Point3<N>) -> bool {
        pt.x >= self.min_pt.x
            && pt.x <= self.max_pt.x
            && pt.y >= self.min_pt.y
            && pt.y <= self.max_pt.y
            && pt.z >= self.min_pt.z
            && pt.z <= self.max_pt.z
    }
}

// =============== 测试

#[cfg(test)]
mod navmesh_astar {
    use super::SegmentIndex;
    use crate::NavMeshMap;
    use nalgebra::Point3;
    use raqote::*;

    #[test]
    fn navmesh_hello() {
        // 初始化 导航网格数据
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),   // 0
            Point3::new(5.0, 0.0, 0.0),   // 1
            Point3::new(0.0, 7.0, 0.0),   // 2
            Point3::new(5.0, 7.0, 0.0),   // 3
            Point3::new(10.0, 7.0, 0.0),  // 4
            Point3::new(5.0, 12.0, 0.0),  // 5
            Point3::new(10.0, 12.0, 0.0), // 6
            Point3::new(26.0, 12.0, 0.0), // 7
            Point3::new(10.0, 21.0, 0.0), // 8
        ];

        let indexs = [
            vec![0, 2, 1],
            vec![1, 2, 3],
            vec![1, 3, 4],
            vec![3, 5, 4],
            vec![4, 5, 6],
            vec![4, 6, 7],
            vec![6, 8, 7],
        ];

        // 创建 导航网格
        let mut map = NavMeshMap::new(points, indexs.as_slice());

        // 寻路
        let start = Point3::new(1.0, 1.0, 0.0);
        let end = Point3::new(9.0, 19.0, 0.0);

        let is_simply_path = true;
        let paths = map
            .find_path(start, end, is_simply_path, 100000, 100000)
            .unwrap();

        // 画图
        let mut painter = Painter::new(50.0, &map.points);

        painter.draw_nav_mesh(&map, Color::new(255, 255, 255, 255));

        painter.draw_finding_paths(&paths, Color::new(255, 0, 255, 0));

        painter.save("navmesh_hello.png");
    }

    #[test]
    fn navmesh_simple() {
        // 初始化 导航网格数据
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(0.0, 7.0, 0.0), // 2
            Point3::new(5.0, 7.0, 0.0),
            Point3::new(10.0, 7.0, 0.0),
            Point3::new(5.0, 12.0, 0.0), // 5
            Point3::new(10.0, 12.0, 0.0),
            Point3::new(26.0, 12.0, 0.0),
            Point3::new(10.0, 21.0, 0.0), // 8
            Point3::new(26.0, 21.0, 0.0),
            Point3::new(31.0, 21.0, 0.0),
            Point3::new(26.0, 26.0, 0.0), // 11
            Point3::new(31.0, 26.0, 0.0),
            Point3::new(36.0, 26.0, 0.0),
            Point3::new(31.0, 35.0, 0.0), // 14
        ];

        let indexs = vec![
            vec![0, 2, 1],
            vec![1, 2, 3],
            vec![1, 3, 4],
            vec![3, 5, 4],
            vec![4, 5, 6],
            vec![5, 8, 6],
            vec![6, 8, 7],
            vec![7, 8, 9],
            vec![7, 9, 10],
            vec![8, 11, 9],
            vec![9, 11, 10],
            vec![10, 11, 12],
            vec![11, 14, 12],
            vec![12, 14, 13],
        ];

        // 创建 导航网格
        let mut map = NavMeshMap::new(points, indexs.as_slice());

        // 寻路
        let start = Point3::new(1.0, 1.0, 0.0);
        let end = Point3::new(34.0, 30.0, 0.0);

        let is_simply_path = true;
        let paths = map
            .find_path(start, end, is_simply_path, 100000, 100000)
            .unwrap();

        // 画图
        let mut painter = Painter::new(20.0, &map.points);

        painter.draw_nav_mesh(&map, Color::new(255, 255, 255, 255));

        painter.draw_finding_paths(&paths, Color::new(255, 0, 255, 0));

        painter.save("navmesh_simple.png");
    }
    
    #[test]
    fn navmesh_testdata() {
        let vdata: [f32; 432] = [-16f32,0.04563978f32,2.5f32,-16.33333f32,0.04563975f32,2.666667f32,-16.33333f32,0.04563975f32,7.666667f32,0f32,0.5456398f32,0f32,0f32,0.5456398f32,0f32,-16f32,0.2956398f32,0f32,-16f32,0.04563975f32,2.5f32,-16.33333f32,0.04563975f32,12.5f32,-16.33333f32,0.04563975f32,17f32,0f32,0.04563975f32,17f32,-16.83333f32,0.04563975f32,12f32,-16.33333f32,0.04563975f32,12.5f32,0f32,0.04563975f32,17f32,0f32,0.5456398f32,0f32,-16.33333f32,0.04563975f32,7.666667f32,-16.83333f32,0.04563975f32,8.166667f32,-42.66667f32,0.04563975f32,8.166667f32,-42.66667f32,0.04563975f32,12f32,-16.83333f32,0.04563975f32,12f32,-16.83333f32,0.04563975f32,8.166667f32,-3f32,0.04563975f32,42.66667f32,0f32,0.04563975f32,42.66667f32,0f32,0.04563975f32,40f32,-3.333332f32,0.04563975f32,38.16667f32,-9.833332f32,0.04563975f32,37.83334f32,-8.833332f32,0.04563975f32,38.83334f32,-4f32,0.04563975f32,37f32,0f32,0.04563975f32,32f32,0f32,0.04563975f32,25.83333f32,-3f32,0.04563975f32,42.66667f32,-3.333332f32,0.04563975f32,38.16667f32,-4f32,0.04563975f32,37.5f32,-8.833332f32,0.04563975f32,38.83334f32,-9f32,0.04563975f32,39.5f32,-8.833332f32,0.04563975f32,38.83334f32,-4f32,0.04563975f32,37.5f32,-4f32,0.04563975f32,37f32,-16f32,1.04564f32,-7.333334f32,-16f32,0.2956398f32,0f32,0f32,0.5456398f32,0f32,0f32,1.212306f32,-11f32,-15f32,1.212306f32,-11f32,-15.5f32,1.212306f32,-11.66667f32,-42.66667f32,1.04564f32,-11.66667f32,-42.66667f32,1.04564f32,-7.833334f32,-16.5f32,1.04564f32,-7.833334f32,-15f32,1.212306f32,-11f32,-16.5f32,1.04564f32,-7.833334f32,-16f32,1.04564f32,-7.333334f32,-15f32,1.212306f32,-11f32,-42.66667f32,1.04564f32,-7.833334f32,-42.66667f32,1.04564f32,-11.66667f32,-44.33333f32,1.04564f32,-11.66667f32,-44.33333f32,1.04564f32,-7.833334f32,-42.66667f32,0.04563975f32,12f32,-42.66667f32,0.04563975f32,8.166667f32,-45.83333f32,0.04563975f32,8.166667f32,-45.83333f32,0.04563975f32,12f32,0f32,0.04563975f32,44.33334f32,0f32,0.04563975f32,42.66667f32,-3f32,0.04563975f32,42.66667f32,26f32,1.04564f32,-30.5f32,26.5f32,1.04564f32,-31.16667f32,26.33333f32,1.04564f32,-31.66667f32,23.33333f32,1.04564f32,-33.83334f32,5.166667f32,1.212306f32,-11f32,11f32,1.04564f32,-12f32,5.166667f32,1.212306f32,-11f32,11f32,1.212306f32,-11.5f32,11f32,1.04564f32,-12f32,12.83333f32,0.2956398f32,0f32,12.83333f32,1.212306f32,-11f32,11.5f32,1.212306f32,-11f32,12.83333f32,0.2956398f32,0f32,11.5f32,1.212306f32,-11f32,11f32,1.212306f32,-11.5f32,5.166667f32,1.212306f32,-11f32,0f32,0.5456398f32,0f32,5.166667f32,1.212306f32,-11f32,0f32,1.212306f32,-11f32,0f32,0.5456398f32,0f32,12.5f32,0.04563975f32,3.333333f32,12.83333f32,0.04563975f32,3.166667f32,12.83333f32,0.2956398f32,0f32,1.833333f32,0.04563975f32,23.16667f32,0f32,0.04563975f32,25.83333f32,0f32,0.04563975f32,32f32,11.33333f32,0.04563975f32,18f32,5.833333f32,0.04563975f32,18.5f32,12.5f32,0.04563975f32,3.333333f32,12.83333f32,0.2956398f32,0f32,0f32,0.5456398f32,0f32,0f32,0.04563975f32,17f32,5.833333f32,0.04563975f32,17f32,11.5f32,0.04563975f32,17.33333f32,12.5f32,0.04563975f32,17f32,12.5f32,0.04563975f32,3.333333f32,5.833333f32,0.04563975f32,17f32,6.166667f32,0.04563975f32,17.33333f32,6.166667f32,0.04563975f32,17.33333f32,5.833333f32,0.04563975f32,18.5f32,11.33333f32,0.04563975f32,18f32,11.5f32,0.04563975f32,17.33333f32,41.33334f32,5.462307f32,42.66667f32,42.5f32,5.712307f32,42.33334f32,41.83334f32,5.378973f32,40.5f32,41.33334f32,5.462307f32,42.66667f32,41.83334f32,5.378973f32,40.5f32,41.83334f32,5.29564f32,39.16667f32,37.83334f32,4.29564f32,39.83334f32,36.83334f32,4.128973f32,40.66667f32,36.83334f32,4.128973f32,40.66667f32,32.16667f32,3.128973f32,42.66667f32,41.33334f32,5.462307f32,42.66667f32,42.66667f32,5.712307f32,36.33334f32,42.66667f32,7.54564f32,23.5f32,37.83334f32,4.29564f32,39.83334f32,41.83334f32,5.29564f32,39.16667f32,0f32,0.04563975f32,42.66667f32,4.833333f32,0.04563975f32,42.66667f32,0f32,0.04563975f32,40f32,4.833333f32,0.04563975f32,42.66667f32,0f32,0.04563975f32,42.66667f32,0f32,0.04563975f32,44.33334f32,8.833334f32,0.04563975f32,49.16667f32,16.66667f32,0.04563975f32,49.16667f32,26.33333f32,2.128973f32,49.16667f32,41.33334f32,5.462307f32,42.66667f32,32.16667f32,3.128973f32,42.66667f32,17.33333f32,0.04563976f32,49.16667f32,17.66667f32,0.2956398f32,53.16667f32,26.33333f32,2.128973f32,49.16667f32,17.33333f32,0.04563975f32,49.16667f32,17.66667f32,0.2956398f32,53.16667f32,17.33333f32,0.04563975f32,49.16667f32,16.66667f32,0.04563975f32,49.16667f32,16.66667f32,0.04563975f32,49.16667f32,8.833334f32,0.04563975f32,49.16667f32,16.33333f32,0.04563975f32,53.5f32,17.66667f32,0.2956398f32,53.16667f32,42.66667f32,7.54564f32,23.5f32,42.66667f32,5.712307f32,36.33334f32,49.16667f32,10.37897f32,14.33333f32,45.5f32,9.712307f32,13.16667f32]
;
        // 初始化 导航网格数据
        let mut points = vec![];
        let count = vdata.len() / 3;
        for i in 0..count {
            let x = vdata[i * 3 + 0];
            let y = vdata[i * 3 + 2];
            let z = 0.;
            points.push(Point3::new(x, y, z));
        }

        let idata: [usize; 216] = [0,1,2,0,2,3,4,5,6,7,8,9,10,11,12,10,12,13,10,13,14,10,14,15,16,17,18,16,18,19,20,21,22,20,22,23,24,25,26,24,26,27,24,27,28,29,30,31,29,31,32,29,32,33,34,35,36,37,38,39,37,39,40,37,40,41,42,43,44,42,44,45,42,45,46,47,48,49,50,51,52,50,52,53,54,55,56,54,56,57,58,59,60,61,62,63,61,63,64,61,64,65,61,65,66,67,68,69,70,71,72,73,74,75,73,75,76,73,76,77,78,79,80,81,82,83,84,85,86,84,86,87,84,87,88,89,90,91,89,91,92,89,92,93,94,95,96,94,96,97,94,97,98,99,100,101,99,101,102,103,104,105,106,107,108,106,108,109,106,109,110,111,112,113,114,115,116,114,116,117,118,119,120,121,122,123,121,123,124,121,124,125,126,127,128,126,128,129,130,131,132,133,134,135,136,137,138,136,138,139,140,141,142,140,142,143];

        let mut indexs = vec![];
        let count = idata.len() / 3;
        for i in 0..count {
            let x = idata[i * 3 + 0];
            let y = idata[i * 3 + 1];
            let z = idata[i * 3 + 2];
            indexs.push(vec![x, y, z]);
        }

        // 创建 导航网格
        let mut map = NavMeshMap::new(points, indexs.as_slice());

        // 寻路
        let start = Point3::new(1.0, 1.0, 0.0);
        let end = Point3::new(47.0, 15.0, 0.0);

        let is_simply_path = false;
        let paths = map
            .find_path(start, end, is_simply_path, 100000, 100000)
            .unwrap();

        // 画图
        let mut painter = Painter::new(20.0, &map.points);

        painter.draw_nav_mesh(&map, Color::new(255, 255, 255, 255));

        painter.draw_finding_paths(&paths, Color::new(255, 0, 255, 0));

        painter.save("navmesh_simple.png");
    }

    #[test]
    fn navmesh_testdata() {
        let vdata: [f32; 432] = [
            -16f32,
            0.04563978f32,
            2.5f32,
            -16.33333f32,
            0.04563975f32,
            2.666667f32,
            -16.33333f32,
            0.04563975f32,
            7.666667f32,
            0f32,
            0.5456398f32,
            0f32,
            0f32,
            0.5456398f32,
            0f32,
            -16f32,
            0.2956398f32,
            0f32,
            -16f32,
            0.04563975f32,
            2.5f32,
            -16.33333f32,
            0.04563975f32,
            12.5f32,
            -16.33333f32,
            0.04563975f32,
            17f32,
            0f32,
            0.04563975f32,
            17f32,
            -16.83333f32,
            0.04563975f32,
            12f32,
            -16.33333f32,
            0.04563975f32,
            12.5f32,
            0f32,
            0.04563975f32,
            17f32,
            0f32,
            0.5456398f32,
            0f32,
            -16.33333f32,
            0.04563975f32,
            7.666667f32,
            -16.83333f32,
            0.04563975f32,
            8.166667f32,
            -42.66667f32,
            0.04563975f32,
            8.166667f32,
            -42.66667f32,
            0.04563975f32,
            12f32,
            -16.83333f32,
            0.04563975f32,
            12f32,
            -16.83333f32,
            0.04563975f32,
            8.166667f32,
            -3f32,
            0.04563975f32,
            42.66667f32,
            0f32,
            0.04563975f32,
            42.66667f32,
            0f32,
            0.04563975f32,
            40f32,
            -3.333332f32,
            0.04563975f32,
            38.16667f32,
            -9.833332f32,
            0.04563975f32,
            37.83334f32,
            -8.833332f32,
            0.04563975f32,
            38.83334f32,
            -4f32,
            0.04563975f32,
            37f32,
            0f32,
            0.04563975f32,
            32f32,
            0f32,
            0.04563975f32,
            25.83333f32,
            -3f32,
            0.04563975f32,
            42.66667f32,
            -3.333332f32,
            0.04563975f32,
            38.16667f32,
            -4f32,
            0.04563975f32,
            37.5f32,
            -8.833332f32,
            0.04563975f32,
            38.83334f32,
            -9f32,
            0.04563975f32,
            39.5f32,
            -8.833332f32,
            0.04563975f32,
            38.83334f32,
            -4f32,
            0.04563975f32,
            37.5f32,
            -4f32,
            0.04563975f32,
            37f32,
            -16f32,
            1.04564f32,
            -7.333334f32,
            -16f32,
            0.2956398f32,
            0f32,
            0f32,
            0.5456398f32,
            0f32,
            0f32,
            1.212306f32,
            -11f32,
            -15f32,
            1.212306f32,
            -11f32,
            -15.5f32,
            1.212306f32,
            -11.66667f32,
            -42.66667f32,
            1.04564f32,
            -11.66667f32,
            -42.66667f32,
            1.04564f32,
            -7.833334f32,
            -16.5f32,
            1.04564f32,
            -7.833334f32,
            -15f32,
            1.212306f32,
            -11f32,
            -16.5f32,
            1.04564f32,
            -7.833334f32,
            -16f32,
            1.04564f32,
            -7.333334f32,
            -15f32,
            1.212306f32,
            -11f32,
            -42.66667f32,
            1.04564f32,
            -7.833334f32,
            -42.66667f32,
            1.04564f32,
            -11.66667f32,
            -44.33333f32,
            1.04564f32,
            -11.66667f32,
            -44.33333f32,
            1.04564f32,
            -7.833334f32,
            -42.66667f32,
            0.04563975f32,
            12f32,
            -42.66667f32,
            0.04563975f32,
            8.166667f32,
            -45.83333f32,
            0.04563975f32,
            8.166667f32,
            -45.83333f32,
            0.04563975f32,
            12f32,
            0f32,
            0.04563975f32,
            44.33334f32,
            0f32,
            0.04563975f32,
            42.66667f32,
            -3f32,
            0.04563975f32,
            42.66667f32,
            26f32,
            1.04564f32,
            -30.5f32,
            26.5f32,
            1.04564f32,
            -31.16667f32,
            26.33333f32,
            1.04564f32,
            -31.66667f32,
            23.33333f32,
            1.04564f32,
            -33.83334f32,
            5.166667f32,
            1.212306f32,
            -11f32,
            11f32,
            1.04564f32,
            -12f32,
            5.166667f32,
            1.212306f32,
            -11f32,
            11f32,
            1.212306f32,
            -11.5f32,
            11f32,
            1.04564f32,
            -12f32,
            12.83333f32,
            0.2956398f32,
            0f32,
            12.83333f32,
            1.212306f32,
            -11f32,
            11.5f32,
            1.212306f32,
            -11f32,
            12.83333f32,
            0.2956398f32,
            0f32,
            11.5f32,
            1.212306f32,
            -11f32,
            11f32,
            1.212306f32,
            -11.5f32,
            5.166667f32,
            1.212306f32,
            -11f32,
            0f32,
            0.5456398f32,
            0f32,
            5.166667f32,
            1.212306f32,
            -11f32,
            0f32,
            1.212306f32,
            -11f32,
            0f32,
            0.5456398f32,
            0f32,
            12.5f32,
            0.04563975f32,
            3.333333f32,
            12.83333f32,
            0.04563975f32,
            3.166667f32,
            12.83333f32,
            0.2956398f32,
            0f32,
            1.833333f32,
            0.04563975f32,
            23.16667f32,
            0f32,
            0.04563975f32,
            25.83333f32,
            0f32,
            0.04563975f32,
            32f32,
            11.33333f32,
            0.04563975f32,
            18f32,
            5.833333f32,
            0.04563975f32,
            18.5f32,
            12.5f32,
            0.04563975f32,
            3.333333f32,
            12.83333f32,
            0.2956398f32,
            0f32,
            0f32,
            0.5456398f32,
            0f32,
            0f32,
            0.04563975f32,
            17f32,
            5.833333f32,
            0.04563975f32,
            17f32,
            11.5f32,
            0.04563975f32,
            17.33333f32,
            12.5f32,
            0.04563975f32,
            17f32,
            12.5f32,
            0.04563975f32,
            3.333333f32,
            5.833333f32,
            0.04563975f32,
            17f32,
            6.166667f32,
            0.04563975f32,
            17.33333f32,
            6.166667f32,
            0.04563975f32,
            17.33333f32,
            5.833333f32,
            0.04563975f32,
            18.5f32,
            11.33333f32,
            0.04563975f32,
            18f32,
            11.5f32,
            0.04563975f32,
            17.33333f32,
            41.33334f32,
            5.462307f32,
            42.66667f32,
            42.5f32,
            5.712307f32,
            42.33334f32,
            41.83334f32,
            5.378973f32,
            40.5f32,
            41.33334f32,
            5.462307f32,
            42.66667f32,
            41.83334f32,
            5.378973f32,
            40.5f32,
            41.83334f32,
            5.29564f32,
            39.16667f32,
            37.83334f32,
            4.29564f32,
            39.83334f32,
            36.83334f32,
            4.128973f32,
            40.66667f32,
            36.83334f32,
            4.128973f32,
            40.66667f32,
            32.16667f32,
            3.128973f32,
            42.66667f32,
            41.33334f32,
            5.462307f32,
            42.66667f32,
            42.66667f32,
            5.712307f32,
            36.33334f32,
            42.66667f32,
            7.54564f32,
            23.5f32,
            37.83334f32,
            4.29564f32,
            39.83334f32,
            41.83334f32,
            5.29564f32,
            39.16667f32,
            0f32,
            0.04563975f32,
            42.66667f32,
            4.833333f32,
            0.04563975f32,
            42.66667f32,
            0f32,
            0.04563975f32,
            40f32,
            4.833333f32,
            0.04563975f32,
            42.66667f32,
            0f32,
            0.04563975f32,
            42.66667f32,
            0f32,
            0.04563975f32,
            44.33334f32,
            8.833334f32,
            0.04563975f32,
            49.16667f32,
            16.66667f32,
            0.04563975f32,
            49.16667f32,
            26.33333f32,
            2.128973f32,
            49.16667f32,
            41.33334f32,
            5.462307f32,
            42.66667f32,
            32.16667f32,
            3.128973f32,
            42.66667f32,
            17.33333f32,
            0.04563976f32,
            49.16667f32,
            17.66667f32,
            0.2956398f32,
            53.16667f32,
            26.33333f32,
            2.128973f32,
            49.16667f32,
            17.33333f32,
            0.04563975f32,
            49.16667f32,
            17.66667f32,
            0.2956398f32,
            53.16667f32,
            17.33333f32,
            0.04563975f32,
            49.16667f32,
            16.66667f32,
            0.04563975f32,
            49.16667f32,
            16.66667f32,
            0.04563975f32,
            49.16667f32,
            8.833334f32,
            0.04563975f32,
            49.16667f32,
            16.33333f32,
            0.04563975f32,
            53.5f32,
            17.66667f32,
            0.2956398f32,
            53.16667f32,
            42.66667f32,
            7.54564f32,
            23.5f32,
            42.66667f32,
            5.712307f32,
            36.33334f32,
            49.16667f32,
            10.37897f32,
            14.33333f32,
            45.5f32,
            9.712307f32,
            13.16667f32,
        ];
        // 初始化 导航网格数据
        let mut points = vec![];
        let count = vdata.len() / 3;
        for i in 0..count {
            let x = vdata[i * 3 + 0];
            let y = vdata[i * 3 + 2];
            let z = 0.;
            points.push(Point3::new(x, y, z));
        }

        let idata: [usize; 216] = [
            0, 1, 2, // 0
            0, 2, 3, // 1
            4, 5, 6, // 2
            7, 8, 9, // 3
            10, 11, 12, // 4
            10, 12, 13, // 5
            10, 13, 14, // 6
            10, 14, 15, // 7
            16, 17, 18, // 8
            16, 18, 19, // 9
            20, 21, 22, // 10
            20, 22, 23, // 11
            24, 25, 26, // 12
            24, 26, 27, // 13
            24, 27, 28, // 14
            29, 30, 31, // 15
            29, 31, 32, // 16
            29, 32, 33, // 17
            34, 35, 36, // 18
            37, 38, 39, // 19
            37, 39, 40, // 20
            37, 40, 41, // 21
            42, 43, 44, // 22
            42, 44, 45, // 23
            42, 45, 46, // 24
            47, 48, 49, // 1
            50, 51, 52, // 1
            50, 52, 53, // 1
            54, 55, 56, // 1
            54, 56, 57, // 1
            58, 59, 60, // 1
            61, 62, 63, // 1
            61, 63, 64, // 1
            61, 64, 65, // 1
            61, 65, 66, // 1
            67, 68, 69, // 1
            70, 71, 72, // 1
            73, 74, 75, // 1
            73, 75, 76, // 1
            73, 76, 77, // 1
            78, 79, 80, // 1
            81, 82, 83, // 1
            84, 85, 86, // 1
            84, 86, 87, // 1
            84, 87, 88, // 1

            89, 90, 91, // 1
            89, 91, 92, // 1
            89, 92, 93, // 1
            
            94, 95, 96, // 1
            94, 96, 97, // 1
            94, 97, 98, // 1
            99, 100, 101, // 1
            99, 101, 102, // 1
            103, 104, 105, // 1
            106, 107, 108, // 1
            106, 108, 109, // 1
            106, 109, 110, // 1
            111, 112, 113, // 1
            114, 115, 116, // 1
            114, 116, 117, // 1
            118, 119, 120, // 1
            121, 122, 123, // 1
            121, 123, 124, // 1
            121, 124, 125, // 1
            126, 127, 128, // 1
            126, 128, 129, // 1
            130, 131, 132, // 1
            133, 134, 135, // 1
            136, 137, 138, // 1
            136, 138, 139, // 1
            140, 141, 142, // 1
            140, 142, 143, // 1
        ];

        let mut indexs = vec![];
        let count = idata.len() / 3;
        for i in 0..count {
            let x = idata[i * 3 + 0];
            let y = idata[i * 3 + 1];
            let z = idata[i * 3 + 2];
            indexs.push(vec![x, y, z]);
        }

        // 创建 导航网格
        let mut map = NavMeshMap::new(points, indexs.as_slice());

        // 寻路
        let start = Point3::new(1.0, 1.0, 0.0);
        let end = Point3::new(-34.0, -9.5, 0.0);

        let is_simply_path = false;
        let paths = map
            .find_path(start, end, is_simply_path, 100000, 100000)
            .unwrap();

        // 画图
        let mut painter = Painter::new(50.0, &map.points);

        painter.draw_nav_mesh(&map, Color::new(255, 255, 255, 255));

        painter.draw_finding_paths(&paths, Color::new(255, 0, 255, 0));

        painter.save("navmesh_testdata.png");
    }

    // 画图方法
    struct Painter {
        dt: DrawTarget,
        scale: f32,

        min_pt: Point3<f32>,
        max_pt: Point3<f32>,
    }

    impl Painter {
        fn new(scale: f32, pts: &[Point3<f32>]) -> Self {
            let mut min_pt = Point3::new(f32::MAX, f32::MAX, 0.0);
            let mut max_pt = Point3::new(f32::MIN, f32::MIN, 0.0);

            for p in pts.iter() {
                min_pt.x = f32::min(min_pt.x, p.x);
                min_pt.y = f32::min(min_pt.y, p.y);

                max_pt.x = f32::max(max_pt.x, p.x);
                max_pt.y = f32::max(max_pt.y, p.y);
            }

            let w = max_pt.x - min_pt.x;
            let h = max_pt.y - min_pt.y;
            let dt = DrawTarget::new((w * scale) as i32, (h * scale) as i32);

            Self {
                scale,

                min_pt,
                max_pt,

                dt,
            }
        }

        fn save(&self, file_name: &str) {
            self.dt.write_png(file_name).unwrap();
        }

        fn draw_nav_mesh(&mut self, map: &NavMeshMap<f32>, color: Color) {
            let mut builder = PathBuilder::new();

            for polygon in map.polygons.iter() {
                self.draw_polygon(&mut builder, map, &polygon.segments);
            }

            let stroke_style = StrokeStyle {
                cap: LineCap::Round,
                join: LineJoin::Round,
                width: 6.,
                miter_limit: 2.,
                dash_array: vec![10., 0.],
                dash_offset: 16.,
            };

            let line_source = Source::Solid(SolidSource::from(color));
            self.dt.stroke(
                &builder.finish(),
                &line_source,
                &stroke_style,
                &DrawOptions::new(),
            );
        }

        fn draw_finding_paths(&mut self, points: &[Point3<f32>], color: Color) {
            let mut builder = PathBuilder::new();

            let mut is_first = true;
            for pt in points.iter() {
                if is_first {
                    is_first = false;

                    builder.move_to(
                        self.scale * (pt.x - self.min_pt.x),
                        self.scale * (pt.y - self.min_pt.y),
                    )
                } else {
                    builder.line_to(
                        self.scale * (pt.x - self.min_pt.x),
                        self.scale * (pt.y - self.min_pt.y),
                    )
                }
            }

            let stroke_style = StrokeStyle {
                cap: LineCap::Round,
                join: LineJoin::Round,
                width: 3.,
                miter_limit: 2.,
                dash_array: vec![10., 0.],
                dash_offset: 16.,
            };

            let path_source = Source::Solid(SolidSource::from(color));
            self.dt.stroke(
                &builder.finish(),
                &path_source,
                &stroke_style,
                &DrawOptions::new(),
            );
        }

        fn draw_polygon(
            &self,
            builder: &mut PathBuilder,
            map: &NavMeshMap<f32>,
            polygon_segments: &[SegmentIndex],
        ) {
            let pts = map.points.as_slice();
            let segments = map.segments.as_slice();

            for index in polygon_segments {
                let segment = &segments[index.0];
                let begin = &pts[segment.start.0];
                let end = &pts[segment.end.0];

                builder.move_to(
                    self.scale * (begin.x - self.min_pt.x),
                    self.scale * (begin.y - self.min_pt.y),
                );

                builder.line_to(
                    self.scale * (end.x - self.min_pt.x),
                    self.scale * (end.y - self.min_pt.y),
                );
            }
        }
    }
}
