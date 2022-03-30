// //!
// //! 基于NavMesh图的A*寻路
// //!

use crate::{AFilter, AStar, AStarMap};
use nalgebra::{Matrix3x1, Point3, RealField};
use num_traits::FromPrimitive;
use std::{collections::HashMap, marker::PhantomData};

type SegmentIndex = usize;

/// ## Nav-Mesh*寻路的寻路主体代理约束
///
/// + 通过代理将不同寻路能力主体的寻路情况交由上层逻辑决定，增加导航图的通用性增加导航图的通用性及寻路能力的动态性
///
/// 需要实现指定两点主体是否能通过，以及通过的代价
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
/// 示例：
/// # Examples
/// ```
/// struct TestFilter {}
/// impl ANavMeshFilter<f64> for TestFilter {
///     fn is_pass(&self, _: Point3<f64>, _: Point3<f64>) -> bool {
///         true
///     }
///     fn get_cost(&self, from: Point3<f64>, to: Point3<f64>) -> f64 {
///         nalgebra::distance(&from, &to)
///     }
/// }
/// ```
pub trait ANavMeshFilter<N>
where
    N: RealField,
{
    /// 主体于两点间是否可通过
    fn is_pass(&self, from: Point3<N>, to: Point3<N>) -> bool;

    /// 主体于两点间通过代价
    fn get_cost(&self, from: Point3<N>, to: Point3<N>) -> N;
}

/// ## Nav-Mesh A* 寻路的Nav-Mesh图
///
/// ### 对`N`的约束
///
/// + 算术运算，可拷贝，可偏序比较， 可与整数相互转换；
/// + 实际使用的时候就是数字类型，比如：i8/i16/i32/f32/f64；
///
pub struct NavMeshMap<'a, N>
where
    N: RealField,
{
    // 该图 所有 线段
    segments: Vec<Segment<'a, N>>,

    // 该图 所有 凸多边形
    polygon: Vec<Polygon<N>>,

    // 调用A*的内部存储结构,支持多线程（一张图同时多个寻路请求）
    astar: Vec<AStar<N>>,
}

impl<'a, N> NavMeshMap<'a, N>
where
    N: RealField,
{
    ///
    ///新建一个NavMesh图
    ///
    /// `vertexs`传入构成Nav-Mesh图的所有顶点坐标的序列
    ///
    /// `indexs`传入指定顶点构成三角形网格，指定方式为`vertexs`中的序列
    ///
    /// 附：Unity中数据导出示例C#代码：
    ///
    /// # Examples
    /// ```
    /// StreamWriter streamWriter = new StreamWriter(path);
    /// streamWriter.WriteLine("v");
    /// List<Vector3> vertexs = new List<Vector3>();
    /// Dictionary<int, int> vertexIndex = new Dictionary<int, int>();
    /// for (int i = 0; i < navMeshTriangulation.vertices.Length; i++)
    /// {
    ///     bool newVert = true;
    ///     for (int ii = 0; ii < vertexs.Count; ii++)
    ///     {
    ///         if ((vertexs[ii] - navMeshTriangulation.vertices[i]).magnitude < 0.0001)
    ///         {
    ///             newVert = false;
    ///             vertexIndex.Add(i, ii);
    ///             break;
    ///         }
    ///     }
    ///     if (newVert)
    ///     {
    ///         vertexIndex.Add(i, vertexs.Count);
    ///         vertexs.Add(navMeshTriangulation.vertices[i]);
    ///         streamWriter.WriteLine(navMeshTriangulation.vertices[i].x + " " + navMeshTriangulation.vertices[i].y + " " + navMeshTriangulation.vertices[i].z);
    ///     }
    /// }
    /// List<List<int>> polygons = new List<List<int>>();
    /// List<int> polygon_types = new List<int>();
    /// for (int i = 0; i < navMeshTriangulation.indices.Length;)
    /// {
    ///     bool merge = false;
    ///     foreach (var polygon in polygons)
    ///     {
    ///         if (polygon.Contains(navMeshTriangulation.indices[i]) && polygon.Contains(navMeshTriangulation.indices[i + 1]))
    ///         {
    ///             polygon.Add(navMeshTriangulation.indices[i + 2]);
    ///             merge = true;
    ///             break;
    ///         }
    ///         if (polygon.Contains(navMeshTriangulation.indices[i + 1]) && polygon.Contains(navMeshTriangulation.indices[i + 2]))
    ///         {
    ///             polygon.Add(navMeshTriangulation.indices[i]);
    ///             merge = true;
    ///             break;
    ///         }
    ///         if (polygon.Contains(navMeshTriangulation.indices[i]) && polygon.Contains(navMeshTriangulation.indices[i + 2]))
    ///         {
    ///             polygon.Add(navMeshTriangulation.indices[i + 1]);
    ///             merge = true;
    ///             break;
    ///         }
    ///     }
    ///     if(!merge)
    ///     {
    ///         List<int> temp = new List<int>();
    ///         temp.Add(navMeshTriangulation.indices[i]);
    ///         temp.Add(navMeshTriangulation.indices[i + 1]);
    ///         temp.Add(navMeshTriangulation.indices[i + 2]);
    ///         polygons.Add(temp);
    ///         polygon_types.Add(navMeshTriangulation.areas[i/3]);
    ///     }
    ///     i = i + 3;
    /// }
    /// streamWriter.WriteLine("f");
    /// for (int i = 0; i < polygons.Count; i++)
    /// {
    ///     string line = "";
    ///     for(int ii = 0; ii < polygons[i].Count; ii++)
    ///     {
    ///         int index = vertexIndex[polygons[i][ii]];
    ///         if (ii != 0)
    ///             line += " " + index.ToString();
    ///         else
    ///             line += index.ToString();
    ///     }
    ///     streamWriter.WriteLine(line);
    /// }
    /// streamWriter.WriteLine("a");
    /// for (int i = 0; i < polygon_types.Count; i++)
    /// {
    ///     streamWriter.WriteLine(navMeshTriangulation.areas[i]);
    /// }
    /// streamWriter.Flush();
    /// streamWriter.Close();
    /// ```
    pub fn new_map(
        vertexs: &[Point3<N>],
        indexs: &[Vec<usize>],
        max_find_num_in_time: usize,
    ) -> Self {
        let mut map = Self {
            segments: Vec::with_capacity(indexs.len() * 2),
            polygon: Vec::with_capacity(indexs.len()),
            astar: Vec::with_capacity(max_find_num_in_time),
        };
        for _ in 0..max_find_num_in_time {
            map.astar.push(AStar::new(indexs.len() * 2));
        }

        //所有实例了的线段，(indexs index, indexs index) -> map index
        let mut all_segments: HashMap<(usize, usize), SegmentIndex> = HashMap::new();
        for (_, index) in indexs.iter().enumerate() {
            //多边形的构建
            let point_num: N = FromPrimitive::from_usize(index.len()).unwrap();
            let mut polygon = Polygon {
                points: Vec::with_capacity(index.len()),
                center: Point3::new(N::zero(), N::zero(), N::zero()),
                segments: Vec::with_capacity(index.len()),
                index: map.polygon.len(),
                min_point: None,
                max_point: None,
            };
            let mut point_x_sum = N::zero();
            let mut point_y_sum = N::zero();
            let mut point_z_sum = N::zero();

            for index_item in index.iter() {
                polygon.points.push(vertexs[*index_item]);
                point_x_sum += vertexs[*index_item].x;
                point_y_sum += vertexs[*index_item].y;
                point_z_sum += vertexs[*index_item].z;
                match polygon.min_point {
                    None => polygon.min_point = Some(vertexs[*index_item]),
                    Some(mut point) => {
                        point.x = point.x.min(vertexs[*index_item].x);
                        point.y = point.y.min(vertexs[*index_item].y);
                        point.z = point.z.min(vertexs[*index_item].z);
                        polygon.min_point = Some(point);
                    }
                }
                match polygon.max_point {
                    None => polygon.max_point = Some(vertexs[*index_item]),
                    Some(mut point) => {
                        point.x = point.x.max(vertexs[*index_item].x);
                        point.y = point.y.max(vertexs[*index_item].y);
                        point.z = point.z.max(vertexs[*index_item].z);
                        polygon.max_point = Some(point);
                    }
                }
            }
            polygon.center = Point3::new(
                point_x_sum / point_num,
                point_y_sum / point_num,
                point_z_sum / point_num,
            );

            //边的构建
            let mut segments_index = Vec::with_capacity(index.len());
            for (i, _) in index.iter().enumerate() {
                let i2 = (i + 1) % index.len();
                match all_segments.get(&(index[i], index[i2])) {
                    None => {
                        match all_segments.get(&(index[i2], index[i])) {
                            None => {
                                //该边没有被实例过
                                let segment = Segment {
                                    center: nalgebra::center(
                                        &vertexs[index[i]],
                                        &vertexs[index[i2]],
                                    ),
                                    point1: vertexs[index[i]],
                                    point2: vertexs[index[i2]],
                                    neighbors: Vec::with_capacity(index.len() * 2),
                                    phantom: PhantomData::default(),
                                };
                                all_segments.insert((index[i], index[i2]), map.segments.len());
                                segments_index.push(map.segments.len());
                                map.segments.push(segment);
                            }
                            Some(index) => {
                                //该边以及实例过
                                segments_index.push(*index);
                            }
                        }
                    }
                    Some(index) => {
                        //该边以及实例过
                        segments_index.push(*index);
                    }
                }
            }

            //该多边形的所有线段增加邻居线段
            for segment_index in segments_index.iter() {
                let segment_ref = map
                    .segments
                    .get_mut(*segment_index)
                    .expect("Get Wrong Segment");
                for segment_index2 in segments_index.iter() {
                    if segment_index2 != segment_index {
                        segment_ref.neighbors.push(*segment_index2);
                    }
                }
            }

            //多边形存储组成线段索引
            for segment_index in segments_index.iter() {
                polygon.segments.push(*segment_index);
            }

            map.polygon.push(polygon);
        }
        map
    }

    ///
    /// 通过A*算法获取寻路路径
    ///
    /// 需要指定场景中的开始坐标及目标坐标，寻路主体代理实现，以及最多访问Nav-Mesh线段的数量,
    ///
    /// 返回为有效路径坐标序列
    ///
    /// + 超出Nav-Mesh图范围的开始点及目标点会被强行拉至Nav-Mesh图范围内
    ///
    pub fn find_path<Filter: ANavMeshFilter<N>>(
        &mut self,
        start_pos: Point3<N>,
        end_pos: Point3<N>,
        filter: &Filter,
        max_nodes: usize,
    ) -> Vec<Point3<N>> {
        //起点多边形
        let start_polygon = self.get_polygon_by_point(&start_pos);
        //起点边
        let start_segment_index =
            self.polygon[start_polygon].get_nearst_segment_index(&start_pos, self);
        //终点多边形
        let end_polygon = self.get_polygon_by_point(&end_pos);
        //终点边
        let end_segment_index = self.polygon[end_polygon].get_nearst_segment_index(&end_pos, self);

        let nav_filter = NavMeshFilter {
            filter: filter,
            segments: &self.segments,
        };
        let mut astar = if self.astar.len() > 0 {
            self.astar.pop().unwrap()
        } else {
            AStar::new(self.segments.len())
        };
        let path_to_end = astar.find_path(
            self,
            start_segment_index,
            end_segment_index,
            &nav_filter,
            max_nodes,
        );

        if astar.result_indies.len() > 1
            && self.polygon[start_polygon]
                .contain_point(self.segments[astar.result_indies[1]].center)
        {
            //由于找最近边为起点边，产生了折现（）路径第一个点和第二个点和起点在同一个多边形内）
            astar.result_indies.remove(0);
        }

        //路径转化为坐标
        let mut path = self.funnel(&mut astar, start_pos);

        if path_to_end {
            path.push(end_pos)
        }
        self.astar.push(astar);
        path
    }
}

// 遍历邻居的迭代器
pub struct ASegmentIterator<'a> {
    neighbors_ref: &'a Vec<usize>,
    cur_index: usize,
}

//============================= 本地
// 对astar的寻路主体代理的封装
struct NavMeshFilter<'a, N, Filter>
where
    N: RealField,
    Filter: ANavMeshFilter<N>,
{
    filter: &'a Filter,
    segments: &'a [Segment<'a, N>],
}

impl<'a, N, Filter> AFilter<N> for NavMeshFilter<'a, N, Filter>
where
    N: RealField,
    Filter: ANavMeshFilter<N>,
{
    // 从`from`节点到`to`节点是否可达
    fn is_pass(&self, from: SegmentIndex, to: SegmentIndex) -> bool {
        self.filter
            .is_pass(self.segments[from].center, self.segments[to].center)
    }

    // 从`parent`节点到当前`cur`节点的`g`，即从父节点到当前节点的实际代价
    fn get_g(&self, cur: usize, parent: usize) -> N {
        self.filter
            .get_cost(self.segments[parent].center, self.segments[cur].center)
    }

    // 从当前`cur`节点到`end`节点的`h`，即从当前节点到终点的预估代价
    fn get_h(&self, cur: usize, end: usize) -> N {
        self.filter
            .get_cost(self.segments[cur].center, self.segments[end].center)
    }
}

impl<'a, N> NavMeshMap<'a, N>
where
    N: RealField,
{
    //获取给定点所在的多边形，若该点不处于任何多边形内，则查找最近多边形
    fn get_polygon_by_point(&self, point: &Point3<N>) -> usize {
        for polygon_ref in self.polygon.iter() {
            if polygon_ref.contain_point(*point) {
                return polygon_ref.index;
            }
        }

        //点不在任何多边形内
        let mut nearst_index: usize = 0;
        let mut nearst_distance_sqr =
            nalgebra::distance_squared(&self.polygon[nearst_index].center, &point);
        for (index, polygon) in self.polygon.iter().enumerate() {
            let distance_sqr = nalgebra::distance_squared(&polygon.center, &point);
            if distance_sqr < nearst_distance_sqr {
                nearst_index = index;
                nearst_distance_sqr = distance_sqr;
            }
        }

        return self.polygon.get(nearst_index).unwrap().index;
    }

    // 检查三个向量是否按顺序排布
    fn vertor_is_order(v1: &Matrix3x1<N>, v2: &Matrix3x1<N>, v3: &Matrix3x1<N>) -> bool {
        let cross1 = v2.cross(&v1);
        let cross2 = v2.cross(&v3);

        return cross1.dot(&cross2) <= N::zero();
    }

    //拉直: 漏斗算法
    //http://liweizhaolili.lofter.com/post/1cc70144_86a939e
    //http://digestingduck.blogspot.hk/2010/03/simple-stupid-funnel-algorithm.html
    fn funnel(&self, astar: &mut AStar<N>, start: Point3<N>) -> Vec<Point3<N>> {
        let mut points: Vec<Point3<N>> = Vec::new();

        if astar.result_indies.len() == 1 {
            return points;
        }

        let mut old_v1 = self.segments[astar.result_indies[0]].point1 - start;
        let mut old_v2 = self.segments[astar.result_indies[0]].point2 - start;
        let mut new_v1;
        let mut new_v2;
        let mut last_point = start;
        points.push(start);
        let mut index = 1;
        while index < astar.result_indies.len() {
            new_v1 = self.segments[astar.result_indies[index]].point1 - last_point;
            new_v2 = self.segments[astar.result_indies[index]].point2 - last_point;
            let mut pos = None;
            if !NavMeshMap::vertor_is_order(&old_v1, &new_v1, &old_v2) {
                //new_v1在old_v1，old_v2范围外
                if NavMeshMap::vertor_is_order(&new_v1, &old_v1, &old_v2) {
                    //靠近old_v1
                    pos = Some(last_point + old_v1);
                } else {
                    //靠近old_v2
                    pos = Some(last_point + old_v2);
                }
            } else if !NavMeshMap::vertor_is_order(&old_v1, &new_v2, &old_v2) {
                //new_v2在old_v1，old_v2范围外
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

            old_v1 = self.segments[astar.result_indies[index]].point1 - last_point;
            old_v2 = self.segments[astar.result_indies[index]].point2 - last_point;
            index += 1;
        }

        points

        //若测试屏蔽funnel算法的A*结果，将函数主体替换为下部分
        // let mut points: Vec<Point3<N>> = Vec::new();
        // for &index in self.astar.result_indies.iter() {
        //     points.push(self.segments[index].center)
        // }
        // points
    }
}

// 线段--Astart的Node
struct Segment<'a, N>
where
    N: RealField,
{
    center: Point3<N>,            // 中点
    point1: Point3<N>,            // 端点1
    point2: Point3<N>,            // 端点2
    neighbors: Vec<SegmentIndex>, // 相邻的边的index

    phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for ASegmentIterator<'a> {
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

impl<'a, N> AStarMap<'a> for NavMeshMap<'a, N>
where
    N: RealField,
{
    type AStartNodeIter = ASegmentIterator<'a>;

    // 获取遍历邻居节点的迭代器
    fn get_neighbors(&'a self, cur: usize, _: Option<usize>) -> ASegmentIterator {
        ASegmentIterator {
            neighbors_ref: &self.segments[cur].neighbors,
            cur_index: 0,
        }
    }
}

// 多边形
struct Polygon<N>
where
    N: RealField,
{
    center: Point3<N>,
    points: Vec<Point3<N>>,
    segments: Vec<SegmentIndex>,
    index: usize,
    min_point: Option<Point3<N>>, //AB框左下点
    max_point: Option<Point3<N>>, //AB框右上点
}

impl<N> Polygon<N>
where
    N: RealField,
{
    //给定点是否在多边形内
    fn contain_point(&self, point: Point3<N>) -> bool {
        if point.x < self.min_point.unwrap().x
            || point.y < self.min_point.unwrap().y
            || point.z < self.min_point.unwrap().z
        {
            return false;
        }

        if point.x > self.max_point.unwrap().x
            || point.y > self.max_point.unwrap().y
            || point.z > self.max_point.unwrap().z
        {
            return false;
        }

        let mut last_cross = None;
        for i in 0..self.points.len() {
            let ii = (i + 1) % self.points.len();
            let v1 = self.points[ii] - self.points[i];
            let v2 = point - self.points[i];
            let cross = v1.cross(&v2);
            if last_cross != None {
                if cross.dot(&last_cross.unwrap()) < N::zero() {
                    return false;
                }
            }
            last_cross = Some(cross);
        }
        true
    }

    //给定点到该多边形距离最小的边的索引
    fn get_nearst_segment_index(&self, point: &Point3<N>, map: &NavMeshMap<N>) -> SegmentIndex {
        let mut nearst_segment_index: SegmentIndex = self.segments[0];
        let center0 = map.segments.get(nearst_segment_index).unwrap().center;
        let mut nearst_segment_distance_sqr = nalgebra::distance_squared(&center0, point);
        for segment_index in self.segments.iter() {
            let center = map.segments.get(*segment_index).unwrap().center;
            let distance_sqr = nalgebra::distance_squared(&center, point);

            if distance_sqr < nearst_segment_distance_sqr {
                nearst_segment_index = *segment_index;
                nearst_segment_distance_sqr = distance_sqr;
            }
        }
        nearst_segment_index
    }
}

// // ============================= 测试用例

#[cfg(test)]
mod navmesh_astar {
    use crate::ANavMeshFilter;
    use nalgebra::Point3;
    use raqote::*;
    #[test]
    fn test_navmesh() {
        let contents =
            std::fs::read_to_string("NavMesh.txt").expect("Something went wrong reading the file");

        let mut read_points = false;
        let mut read_indexs = false;
        let mut vertexs: Vec<Point3<f64>> = Vec::new();
        let mut indexs: Vec<Vec<usize>> = Vec::new();
        for line in contents.lines() {
            if line == "v" {
                read_points = true;
                read_indexs = false;
            } else if line == "f" {
                read_indexs = true;
                read_points = false;
            } else {
                if read_points {
                    let mut x = 0.0;
                    let mut y = 0.0;
                    let mut z = 0.0;
                    for (i, contant) in line.split(" ").enumerate() {
                        if i == 0 {
                            x = contant.parse().unwrap();
                        } else if i == 1 {
                            y = contant.parse().unwrap();
                        } else if i == 2 {
                            z = contant.parse().unwrap();
                        }
                    }
                    vertexs.push(Point3::new(x, y, z));
                } else if read_indexs {
                    let mut temp = Vec::new();
                    for contant in line.split(" ") {
                        temp.push(contant.parse().unwrap());
                    }
                    indexs.push(temp);
                }
            }
        }
        let mut map = crate::nav_mesh::NavMeshMap::new_map(&vertexs, &indexs, 5);
        let end = Point3::new(614.0, 320.63, 696.0);
        let start = Point3::new(1373.1, 307.43, 2729.8);

        struct TestFilter {}

        impl ANavMeshFilter<f64> for TestFilter {
            fn is_pass(&self, _: Point3<f64>, _: Point3<f64>) -> bool {
                true
            }

            fn get_cost(&self, from: Point3<f64>, to: Point3<f64>) -> f64 {
                nalgebra::distance(&from, &to)
            }
        }
        let test = TestFilter {};
        let path = map.find_path(start, end, &test, 99999);
        let mut min_x = f64::MAX;
        let mut max_x = 0.0;
        let mut min_z = f64::MAX;
        let mut max_z = 0.0;
        for vertex in vertexs.iter() {
            if min_x > vertex.x {
                min_x = vertex.x;
            }
            if max_x < vertex.x {
                max_x = vertex.x;
            }
            if min_z > vertex.z {
                min_z = vertex.z;
            }
            if max_z < vertex.z {
                max_z = vertex.z;
            }
        }
        let scale = 0.1;
        let mut dt = DrawTarget::new(
            ((max_x - min_x) * scale) as i32,
            ((max_z - min_z) * scale) as i32,
        );
        let line_source = Source::Solid(SolidSource::from(Color::new(255, 0, 0, 0)));
        let mut path_builder = PathBuilder::new();
        let draw_opt = DrawOptions::new();
        let stroke_style = StrokeStyle {
            cap: LineCap::Round,
            join: LineJoin::Round,
            width: 2.,
            miter_limit: 2.,
            dash_array: vec![10., 0.],
            dash_offset: 16.,
        };

        for pol_vertx_idxs in indexs {
            for (iii, _) in pol_vertx_idxs.iter().enumerate() {
                let point = vertexs.get(pol_vertx_idxs[iii]).unwrap();
                if iii == 0 {
                    path_builder.move_to(
                        ((point.x - min_x) * scale) as f32,
                        ((point.z - min_z) * scale) as f32,
                    );
                } else {
                    path_builder.line_to(
                        ((point.x - min_x) * scale) as f32,
                        ((point.z - min_z) * scale) as f32,
                    );
                }
            }
            let point = vertexs.get(pol_vertx_idxs[0]).unwrap();
            path_builder.line_to(
                ((point.x - min_x) * scale) as f32,
                ((point.z - min_z) * scale) as f32,
            );
        }
        let mut draw_path = path_builder.finish();
        dt.stroke(&draw_path, &line_source, &stroke_style, &draw_opt);

        let path_source = Source::Solid(SolidSource::from(Color::new(255, 0, 255, 0)));
        let mut path_builder = PathBuilder::new();
        path_builder.move_to(
            ((start.x - min_x) * scale) as f32,
            ((start.z - min_z) * scale) as f32,
        );
        for path_point in path {
            path_builder.line_to(
                ((path_point.x - min_x) * scale) as f32,
                ((path_point.z - min_z) * scale) as f32,
            );
        }
        path_builder.line_to(
            ((end.x - min_x) * scale) as f32,
            ((end.z - min_z) * scale) as f32,
        );
        draw_path = path_builder.finish();
        dt.stroke(&draw_path, &path_source, &stroke_style, &draw_opt);
        dt.write_png("navmesh_path.png");
    }

    #[test]
    fn test_funnel() {
        use crate::ANavMeshFilter;
        use nalgebra::Point3;
        use raqote::*;
        let contents = std::fs::read_to_string("FunnelTest.txt")
            .expect("Something went wrong reading the file");

        let mut read_points = false;
        let mut read_indexs = false;
        let mut vertexs: Vec<Point3<f64>> = Vec::new();
        let mut indexs: Vec<Vec<usize>> = Vec::new();
        for line in contents.lines() {
            if line == "v" {
                read_points = true;
                read_indexs = false;
            } else if line == "f" {
                read_indexs = true;
                read_points = false;
            } else {
                if read_points {
                    let mut x = 0.0;
                    let mut y = 0.0;
                    let mut z = 0.0;
                    for (i, contant) in line.split(" ").enumerate() {
                        if i == 0 {
                            x = contant.parse().unwrap();
                        } else if i == 1 {
                            y = contant.parse().unwrap();
                        } else if i == 2 {
                            z = contant.parse().unwrap();
                        }
                    }
                    vertexs.push(Point3::new(x, y, z));
                } else if read_indexs {
                    let mut temp = Vec::new();
                    for contant in line.split(" ") {
                        temp.push(contant.parse().unwrap());
                    }
                    indexs.push(temp);
                }
            }
        }
        let mut map = crate::nav_mesh::NavMeshMap::new_map(&vertexs, &indexs, 5);
        let end = Point3::new(373., 0.0, 372.);
        let start = Point3::new(68.6, 0.0, 65.);

        struct TestFilter {}

        impl ANavMeshFilter<f64> for TestFilter {
            fn is_pass(&self, _: Point3<f64>, _: Point3<f64>) -> bool {
                true
            }

            fn get_cost(&self, from: Point3<f64>, to: Point3<f64>) -> f64 {
                nalgebra::distance(&from, &to)
            }
        }
        let test = TestFilter {};
        let path = map.find_path(start, end, &test, 99999);
        let mut min_x = f64::MAX;
        let mut max_x = 0.0;
        let mut min_z = f64::MAX;
        let mut max_z = 0.0;
        for vertex in vertexs.iter() {
            if min_x > vertex.x {
                min_x = vertex.x;
            }
            if max_x < vertex.x {
                max_x = vertex.x;
            }
            if min_z > vertex.z {
                min_z = vertex.z;
            }
            if max_z < vertex.z {
                max_z = vertex.z;
            }
        }
        let scale = 1.0;
        let mut dt = DrawTarget::new(
            ((max_x - min_x) * scale) as i32,
            ((max_z - min_z) * scale) as i32,
        );
        let line_source = Source::Solid(SolidSource::from(Color::new(255, 0, 0, 0)));
        let mut path_builder = PathBuilder::new();
        let draw_opt = DrawOptions::new();
        let stroke_style = StrokeStyle {
            cap: LineCap::Round,
            join: LineJoin::Round,
            width: 2.,
            miter_limit: 2.,
            dash_array: vec![10., 0.],
            dash_offset: 16.,
        };

        for pol_vertx_idxs in indexs {
            for (iii, _) in pol_vertx_idxs.iter().enumerate() {
                let point = vertexs.get(pol_vertx_idxs[iii]).unwrap();
                if iii == 0 {
                    path_builder.move_to(
                        ((point.x - min_x) * scale) as f32,
                        ((point.z - min_z) * scale) as f32,
                    );
                } else {
                    path_builder.line_to(
                        ((point.x - min_x) * scale) as f32,
                        ((point.z - min_z) * scale) as f32,
                    );
                }
            }
            let point = vertexs.get(pol_vertx_idxs[0]).unwrap();
            path_builder.line_to(
                ((point.x - min_x) * scale) as f32,
                ((point.z - min_z) * scale) as f32,
            );
        }
        let mut draw_path = path_builder.finish();
        dt.stroke(&draw_path, &line_source, &stroke_style, &draw_opt);

        let path_source = Source::Solid(SolidSource::from(Color::new(255, 0, 255, 0)));
        let mut path_builder = PathBuilder::new();
        path_builder.move_to(
            ((start.x - min_x) * scale) as f32,
            ((start.z - min_z) * scale) as f32,
        );
        for path_point in path {
            path_builder.line_to(
                ((path_point.x - min_x) * scale) as f32,
                ((path_point.z - min_z) * scale) as f32,
            );
        }
        path_builder.line_to(
            ((end.x - min_x) * scale) as f32,
            ((end.z - min_z) * scale) as f32,
        );
        draw_path = path_builder.finish();
        dt.stroke(&draw_path, &path_source, &stroke_style, &draw_opt);
        dt.write_png("funneltest.png");
    }
}

#[cfg(test)]
mod navtests {
    use crate::ANavMeshFilter;
    use nalgebra::Point3;
    use test::Bencher;

    #[bench]
    fn bench_test(b: &mut Bencher) {
        let contents =
            std::fs::read_to_string("NavMesh.txt").expect("Something went wrong reading the file");

        let mut read_points = false;
        let mut read_indexs = false;
        let mut vertexs: Vec<Point3<f64>> = Vec::new();
        let mut indexs: Vec<Vec<usize>> = Vec::new();
        for line in contents.lines() {
            if line == "v" {
                read_points = true;
                read_indexs = false;
            } else if line == "f" {
                read_indexs = true;
                read_points = false;
            } else {
                if read_points {
                    let mut x = 0.0;
                    let mut y = 0.0;
                    let mut z = 0.0;
                    for (i, contant) in line.split(" ").enumerate() {
                        if i == 0 {
                            x = contant.parse().unwrap();
                        } else if i == 1 {
                            y = contant.parse().unwrap();
                        } else if i == 2 {
                            z = contant.parse().unwrap();
                        }
                    }
                    vertexs.push(Point3::new(x, y, z));
                } else if read_indexs {
                    let mut temp = Vec::new();
                    for contant in line.split(" ") {
                        temp.push(contant.parse().unwrap());
                    }
                    indexs.push(temp);
                }
            }
        }
        let mut map = crate::nav_mesh::NavMeshMap::new_map(&vertexs, &indexs, 5);
        let end = Point3::new(614.0, 320.63, 696.0);
        let start = Point3::new(1373.1, 307.43, 2729.8);

        struct TestFilter {}

        impl ANavMeshFilter<f64> for TestFilter {
            fn is_pass(&self, _: Point3<f64>, _: Point3<f64>) -> bool {
                true
            }

            fn get_cost(&self, from: Point3<f64>, to: Point3<f64>) -> f64 {
                nalgebra::distance(&from, &to)
            }
        }
        let test = TestFilter {};
        b.iter(|| map.find_path(start, end, &test, 99999));
    }
}
