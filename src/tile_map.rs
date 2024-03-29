//!
//! A*寻路的瓦片地图
//! 需指定行数和列数，每个瓦片的边长和斜角长度，默认为100和141。
//! 瓦片信息包括3个信息：瓦片是否障碍， 右边是否障碍，下边是否障碍。这些是从左到右，从上到下地进行排序的。一个矩阵表示的地图也称为瓦片地图。
//! 获得邻居时， 如果斜向对应的两个直方向的瓦片有1个不可走，则该斜方向就不可走。
//! 如果使用PathSmoothIter来平滑路径，则斜向对应的两个直方向的瓦片有1个不可走，则该斜方向就不可走。
//!

use crate::{
    base::{Aabb, Angle, Point},
    bresenham::Bresenham,
    finder::{AStar, NodeEntry, NodeIndex, ResultIterator, AStarResult},
    normal::{make_neighbors, Entry, Map},
};
use num_traits::Zero;
use pi_null::Null;
use std::fmt::Debug;
use std::mem::{replace, transmute, MaybeUninit};

// 四方向枚举
#[repr(C)]
#[derive(Debug, Clone)]
pub enum Location {
    UpLeft = 0,
    UpRight = 1,
    DownLeft = 2,
    DownRight = 3,
}
#[derive(Debug, Clone)]
pub enum TileFlagType {
    Center = 1,
    Right = 2,
    Down = 4,
}

/// 按到P点的距离从近到远进行排序
pub fn sort_by_dist(p: Point, vec: &mut Vec<Point>) {
    vec.sort_by(|a, b| {
        let x = a.x - p.x;
        let y = a.y - p.y;
        let dist_a = x * x + y * y;
        let x = b.x - p.x;
        let y = b.y - p.y;
        let dist_b = x * x + y * y;
        dist_a.cmp(&dist_b)
    });
}
/// 获得指定位置瓦片及方向的包含自身及周围的四个瓦片坐标，第一个坐标一定是min_xy，最后一个坐标是max_xy，顺序遵循0(左上) 1(右上) 2(左下) 3(右下)
pub fn get_round(p: Point, d: Location, width: isize, height: isize) -> ([Point; 4], usize) {
    // 创建一个长度为 4 的未初始化的数组
    let array: [MaybeUninit<Point>; 4] = unsafe { MaybeUninit::uninit().assume_init() };
    let mut arr: [Point; 4] = unsafe { transmute::<[MaybeUninit<Point>; 4], [Point; 4]>(array) };
    let mut i = 0;
    fn push(arr: &mut [Point], i: &mut usize, p: Point) {
        arr[*i] = p;
        *i += 1;
    }
    match d {
        Location::UpLeft => {
            if p.x > 0 {
                if p.y > 0 {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y - 1)); // 0
                    push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 1
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 2
                } else {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 2
                }
            } else if p.y > 0 {
                push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 1
            }
            push(&mut arr, &mut i, p); // 3
        }
        Location::UpRight => {
            if p.x + 1 < width {
                if p.y > 0 {
                    push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 0
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y - 1)); // 1
                    push(&mut arr, &mut i, p); // 2
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 3
                } else {
                    push(&mut arr, &mut i, p); // 2
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 3
                }
            } else if p.y > 0 {
                push(&mut arr, &mut i, Point::new(p.x, p.y - 1)); // 0
                push(&mut arr, &mut i, p); // 2
            } else {
                push(&mut arr, &mut i, p); // 2
            }
        }
        Location::DownLeft => {
            if p.x > 0 {
                if p.y + 1 < height {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 0
                    push(&mut arr, &mut i, p); // 1
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y + 1)); // 2
                    push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 3
                } else {
                    push(&mut arr, &mut i, Point::new(p.x - 1, p.y)); // 0
                    push(&mut arr, &mut i, p); // 1
                }
            } else if p.y + 1 < height {
                push(&mut arr, &mut i, p); // 1
                push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 3
            } else {
                push(&mut arr, &mut i, p); // 1
            }
        }
        Location::DownRight => {
            push(&mut arr, &mut i, p); // 0
            if p.x + 1 < width {
                if p.y + 1 < height {
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 1
                    push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 2
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y + 1)); // 3
                } else {
                    push(&mut arr, &mut i, Point::new(p.x + 1, p.y)); // 1
                }
            } else if p.y + 1 < height {
                push(&mut arr, &mut i, Point::new(p.x, p.y + 1)); // 2
            }
        }
    }
    return (arr, i);
}

// 八方向枚举
#[repr(C)]
#[derive(Debug, Clone)]
pub enum Direction {
    Left = 0,
    Right = 1,
    Up = 2,
    Down = 3,
    UpLeft = 4,
    UpRight = 5,
    DownLeft = 6,
    DownRight = 7,
}
/// 获得指定位置瓦片的上下左右四个瓦片， 如果为数组元素为null，则超出边界
pub fn get_4d_neighbors(tile_index: usize, width: usize, amount: usize) -> [usize; 4] {
    let mut arr = [Null::null(), Null::null(), Null::null(), Null::null()];
    if tile_index >= amount + width {
        return arr;
    }
    let x = tile_index % width;
    if x == width - 1 {
        arr[Direction::Left as usize] = tile_index - 1;
    } else if x > 0 {
        arr[Direction::Left as usize] = tile_index - 1;
        arr[Direction::Right as usize] = tile_index + 1;
    } else {
        arr[Direction::Right as usize] = tile_index + 1;
    }
    if tile_index + width >= amount {
        arr[Direction::Up as usize] = tile_index - width;
    } else if tile_index >= width {
        arr[Direction::Up as usize] = tile_index - width;
        arr[Direction::Down as usize] = tile_index + width;
    } else {
        arr[Direction::Down as usize] = tile_index + width;
    }
    arr
}
/// 获得指定位置瓦片周围的八个瓦片， 如果为数组元素为null，则超出边界
pub fn get_8d_neighbors(tile_index: usize, width: usize, amount: usize) -> [usize; 8] {
    let mut arr = [
        Null::null(),
        Null::null(),
        Null::null(),
        Null::null(),
        Null::null(),
        Null::null(),
        Null::null(),
        Null::null(),
    ];
    if tile_index >= amount + width {
        return arr;
    }
    let x = tile_index % width;
    if tile_index >= amount {
        arr[Direction::Up as usize] = tile_index - width;
        if x == width - 1 {
            arr[Direction::UpLeft as usize] = arr[Direction::Up as usize] - 1;
        } else if x > 0 {
            arr[Direction::UpLeft as usize] = arr[Direction::Up as usize] - 1;
            arr[Direction::UpRight as usize] = arr[Direction::Up as usize] + 1;
        } else {
            arr[Direction::UpRight as usize] = arr[Direction::Up as usize] + 1;
        }
        return arr;
    }
    if x == width - 1 {
        arr[Direction::Left as usize] = tile_index - 1;
        if tile_index + width >= amount {
            arr[Direction::Up as usize] = tile_index - width;
            arr[Direction::UpLeft as usize] = arr[Direction::Up as usize] - 1;
        } else if tile_index >= width {
            arr[Direction::Up as usize] = tile_index - width;
            arr[Direction::Down as usize] = tile_index + width;
            arr[Direction::UpLeft as usize] = arr[Direction::Up as usize] - 1;
            arr[Direction::DownLeft as usize] = arr[Direction::Down as usize] - 1;
        } else {
            arr[Direction::Down as usize] = tile_index + width;
            arr[Direction::DownLeft as usize] = arr[Direction::Down as usize] - 1;
        }
    } else if x > 0 {
        arr[Direction::Left as usize] = tile_index - 1;
        arr[Direction::Right as usize] = tile_index + 1;
        if tile_index + width >= amount {
            arr[Direction::Up as usize] = tile_index - width;
            arr[Direction::UpLeft as usize] = arr[Direction::Up as usize] - 1;
            arr[Direction::UpRight as usize] = arr[Direction::Up as usize] + 1;
        } else if tile_index >= width {
            arr[Direction::Up as usize] = tile_index - width;
            arr[Direction::Down as usize] = tile_index + width;
            arr[Direction::UpLeft as usize] = arr[Direction::Up as usize] - 1;
            arr[Direction::UpRight as usize] = arr[Direction::Up as usize] + 1;
            arr[Direction::DownLeft as usize] = arr[Direction::Down as usize] - 1;
            arr[Direction::DownRight as usize] = arr[Direction::Down as usize] + 1;
        } else {
            arr[Direction::Down as usize] = tile_index + width;
            arr[Direction::DownLeft as usize] = arr[Direction::Down as usize] - 1;
            arr[Direction::DownRight as usize] = arr[Direction::Down as usize] + 1;
        }
    } else {
        arr[Direction::Right as usize] = tile_index + 1;
        if tile_index + width >= amount {
            arr[Direction::Up as usize] = tile_index - width;
            arr[Direction::UpRight as usize] = arr[Direction::Up as usize] + 1;
        } else if tile_index >= width {
            arr[Direction::Up as usize] = tile_index - width;
            arr[Direction::Down as usize] = tile_index + width;
            arr[Direction::UpRight as usize] = arr[Direction::Up as usize] + 1;
            arr[Direction::DownRight as usize] = arr[Direction::Down as usize] + 1;
        } else {
            arr[Direction::Down as usize] = tile_index + width;
            arr[Direction::DownRight as usize] = arr[Direction::Down as usize] + 1;
        }
    }
    arr
}

const CENTER_MASK: u8 = 0b1111;
const SIDE_MASK: u8 = 0b11;
const RIGHT_INDEX: usize = 4;
const RIGHT_MASK: u8 = 0b110000;
const DOWN_INDEX: usize = 6;
const DOWN_MASK: u8 = 0b11000000;

pub struct TileMap {
    // 该图所有节点是否有障碍， 右边是否障碍，下边是否障碍
    pub nodes: Vec<u8>,
    // 该图宽度
    pub width: usize,
    // 该图高度
    pub height: usize,
    // 该图节点的长度， 如果是整数，则必须大于10
    cell_len: usize,
    // 该图节点间斜45°的长度， 根号2 * cell_len
    oblique_len: usize,
    // 瓦片总数量
    amount: usize,
}

impl TileMap {
    ///
    /// 新建一个瓦片地图
    ///
    /// 需指定瓦片地图的宽度和高度，每个瓦片的边长和斜变长
    pub fn new(width: usize, height: usize, cell_len: usize, oblique_len: usize) -> Self {
        let amount = height * width;
        let mut nodes = Vec::with_capacity(amount);
        nodes.resize_with(amount, Default::default);
        TileMap {
            nodes,
            width,
            height,
            cell_len,
            oblique_len,
            amount: height * width,
        }
    }
    #[inline]
    pub fn get_node_flag(&self, node: NodeIndex) -> u8 {
        if node.0 < self.amount {
            unsafe { *self.nodes.get_unchecked(node.0) }
        } else {
            0
        }
    }
    pub fn get_node_flag_type(&self, node: NodeIndex, flag_type: TileFlagType) -> u8 {
        if node.0 < self.amount {
            let i = unsafe { *self.nodes.get_unchecked(node.0) };
            match flag_type {
                TileFlagType::Center => i & CENTER_MASK,
                TileFlagType::Right => (i >> RIGHT_INDEX) & SIDE_MASK,
                TileFlagType::Down => i >> DOWN_INDEX,
            }
        } else {
            0
        }
    }
    pub fn set_node_flag(&mut self, node: NodeIndex, flag: u8) -> u8 {
        if node.0 < self.amount {
            unsafe { replace(self.nodes.get_unchecked_mut(node.0), flag) }
        } else {
            0
        }
    }
    pub fn set_node_flag_type(
        &mut self,
        node: NodeIndex,
        flag_type: TileFlagType,
        flag_value: u8,
    ) -> u8 {
        if node.0 < self.amount {
            let i = unsafe { *self.nodes.get_unchecked(node.0) };
            match flag_type {
                TileFlagType::Center => {
                    unsafe {
                        *self.nodes.get_unchecked_mut(node.0) =
                            (i & !CENTER_MASK) | (flag_value & CENTER_MASK)
                    };
                    i & CENTER_MASK
                }
                TileFlagType::Right => {
                    unsafe {
                        *self.nodes.get_unchecked_mut(node.0) =
                            (i & !RIGHT_MASK) | ((flag_value & SIDE_MASK) << RIGHT_INDEX)
                    };
                    (i >> RIGHT_INDEX) & SIDE_MASK
                }
                TileFlagType::Down => {
                    unsafe {
                        *self.nodes.get_unchecked_mut(node.0) =
                            (i & !DOWN_MASK) | ((flag_value & SIDE_MASK) << DOWN_INDEX)
                    };
                    i >> DOWN_INDEX
                }
            }
        } else {
            0
        }
    }
    pub fn set_range_flag(&mut self, aabb: &Aabb, flag: u8) {
        let x1 = if aabb.min.x < 0 {
            0
        } else {
            aabb.min.x as usize
        };
        let x2 = if aabb.max.x as usize > self.width {
            self.width
        } else {
            aabb.max.x as usize
        };
        let y1 = if aabb.min.y < 0 {
            0
        } else {
            aabb.min.y as usize
        };
        let y2 = if aabb.max.y as usize > self.height {
            self.height
        } else {
            aabb.max.y as usize
        };
        let mut start = y1 * self.width;
        for _ in y1..y2 {
            for x in x1..x2 {
                unsafe { *self.nodes.get_unchecked_mut(x + start) = flag };
            }
            start += self.width;
        }
    }
    pub fn is_node_center_obstacle(&self, node: NodeIndex, flag: u8) -> bool {
        self.get_node_flag(node) & CENTER_MASK & flag == 0
    }
    pub fn get_node_center_flag(&self, node: NodeIndex) -> u8 {
        self.get_node_flag(node) & CENTER_MASK
    }
    pub fn set_node_center_flag(&mut self, node: NodeIndex, value: u8) -> bool {
        if node.0 >= self.amount {
            return false;
        }
        let flag = unsafe { *self.nodes.get_unchecked(node.0) };
        unsafe {
            *self.nodes.get_unchecked_mut(node.0) = (flag & !CENTER_MASK) | (value & CENTER_MASK)
        };
        true
    }
    pub fn move_center_flag(&mut self, src: NodeIndex, dest: NodeIndex) {
        let flag = self.get_node_center_flag(src);
        self.set_node_center_flag(src, 0);
        self.set_node_center_flag(dest, flag);
    }

    // 获得指定点周围所有可用的点，周围是顺时针一圈一圈扩大，直到可用点数超过count, spacing为可用点的间隔
    pub fn find_round(
        &self,
        node: NodeIndex,
        count: usize,
        spacing: usize,
        d: Direction,
        flag: u8,
        result: &mut Vec<Point>,
    ) -> Aabb {
        let p = get_xy(self.width, node);
        if self.is_node_center_obstacle(node, flag) {
            return Aabb::new(p, p);
        }
        self.find(p, count, (spacing + 1) as isize, flag, d, result)
    }
    // 获得指定点周围所有可用的点，周围是顺时针一圈一圈扩大，直到可用点数超过count, spacing为可用点的间隔
    fn find(
        &self,
        p: Point,
        count: usize,
        spacing: isize,
        flag: u8,
        mut d: Direction,
        result: &mut Vec<Point>,
    ) -> Aabb {
        if count <= 1 {
            // 如果只查找1个点，则可以返回当前点
            result.push(p);
            return Aabb::new(p, p.add(spacing));
        }
        let mut aabb = Aabb::new(p, p);
        if count >= 4 {
            // 先扩大到数量对应的范围上
            let size = (count as f32).sqrt() as isize;
            // 防止尺寸超过宽高
            let mut rsize = size * spacing;
            if rsize > self.width as isize {
                rsize = self.width as isize;
            }
            if rsize > self.height as isize {
                rsize = self.height as isize;
            }
            aabb.min = aabb.min.add(-rsize / 2);
            // 循环增大矩形时，边长偶数表示加了一半，所以需要调整方向和矩形位置
            if size % 2 == 0 {
                match d {
                    Direction::Left => {
                        d = Direction::Right;
                        aabb.min.y += spacing;
                    }
                    Direction::Right => {
                        d = Direction::Left;
                        aabb.min.x += spacing;
                    }
                    Direction::Up => {
                        d = Direction::Down;
                        aabb.min = aabb.min.add(spacing);
                    }
                    _ => {
                        d = Direction::Up;
                    }
                }
            }
            aabb.max = aabb.min.add(rsize);
            // 检查是否超出边界， 超出边界，则移动新范围
            if aabb.min.x < 0 {
                aabb.max.x -= aabb.min.x;
                aabb.min.x = 0;
            }
            if aabb.min.y < 0 {
                aabb.max.y -= aabb.min.y;
                aabb.min.y = 0;
            }
            if aabb.max.x > self.width as isize {
                aabb.min.x -= aabb.max.x - self.width as isize;
                aabb.max.x = self.width as isize;
            }
            if aabb.max.y > self.height as isize {
                aabb.min.y -= aabb.max.y - self.height as isize;
                aabb.max.y = self.height as isize;
            }
            // 将范围的可用点记录下来
            self.spacing_points(spacing, &aabb, flag, result);
            if result.len() >= count {
                return aabb;
            }
        } else {
            aabb.max = p.add(spacing);
            if aabb.max.x > self.width as isize {
                aabb.max.x = self.width as isize;
            }
            if aabb.max.y > self.height as isize {
                aabb.max.y = self.height as isize;
            }
            result.push(p);
        }
        loop {
            // 根据Direction方向，顺时针扩大边界
            // 检查新范围是否超出边界
            // 超出边界，则移动新范围，如果移动后另一端也超出边界，则返回
            // 新范围和旧范围，返回差值范围
            let diff = match d {
                Direction::Left => {
                    // 向左扩展边界
                    d = Direction::Up;
                    if aabb.min.x < spacing {
                        // 超出边界，向右移动范围
                        let old = aabb.max.x;
                        aabb.max.x += spacing;
                        if aabb.max.x > self.width as isize {
                            // 移动后另一端超出边界，则返回
                            return aabb;
                        }
                        Aabb::new(Point::new(old, aabb.min.y), aabb.max)
                    } else {
                        let old = aabb.min.x;
                        aabb.min.x -= spacing; // 向左移动边界
                        Aabb::new(aabb.min, Point::new(old, aabb.max.y))
                    }
                }
                Direction::Right => {
                    // 向右扩展边界
                    d = Direction::Down;
                    if aabb.max.x + spacing > self.width as isize {
                        // 超出边界，向左移动范围
                        let old = aabb.min.x;
                        aabb.min.x -= spacing;
                        if aabb.min.x < 0 {
                            // 移动后另一端超出边界，则返回
                            return aabb;
                        }
                        Aabb::new(aabb.min, Point::new(old, aabb.max.y))
                    } else {
                        let old = aabb.max.x;
                        aabb.max.x += spacing; // 向右移动边界
                        Aabb::new(Point::new(old, aabb.min.y), aabb.max)
                    }
                }
                Direction::Up => {
                    d = Direction::Right;
                    if aabb.max.y + spacing > self.height as isize {
                        // 超出边界，向下移动范围
                        let old = aabb.min.y;
                        aabb.min.y -= spacing;
                        if aabb.min.y < 0 {
                            // 移动后另一端超出边界，则返回
                            return aabb;
                        }
                        Aabb::new(aabb.min, Point::new(aabb.max.x, old))
                    } else {
                        let old = aabb.max.y;
                        aabb.max.y += spacing; // 向上移动边界
                        Aabb::new(Point::new(aabb.min.x, old), aabb.max)
                    }
                }
                _ => {
                    d = Direction::Left;
                    if aabb.min.y < spacing {
                        // 超出边界，向上移动范围
                        let old = aabb.max.y;
                        aabb.max.y += spacing;
                        if aabb.max.y > self.height as isize {
                            // 移动后另一端超出边界，则返回
                            return aabb;
                        }
                        Aabb::new(Point::new(aabb.min.x, old), aabb.max)
                    } else {
                        let old = aabb.min.y;
                        aabb.min.y -= spacing; // 向下移动边界
                        Aabb::new(aabb.min, Point::new(aabb.max.x, old))
                    }
                }
            };
            // 将差值范围的可用点记录下来
            self.spacing_points(spacing, &diff, flag, result);
            if result.len() >= count {
                return aabb;
            }
        }
    }
    // 按照间隔收集可用的点, 外部保证范围是不超出边界的
    fn spacing_points(&self, spacing: isize, aabb: &Aabb, flag: u8, vec: &mut Vec<Point>) {
        let spacing_width = spacing * self.width as isize;
        let mut y = aabb.min.y;
        let mut line_index = y * self.width as isize;
        while y < aabb.max.y {
            let mut x = aabb.min.x;
            while x < aabb.max.x {
                let p = Point::new(x, y);
                self.range_point(line_index, Aabb::new(p, p.add(spacing)), flag, vec);
                x += spacing;
            }
            y += spacing;
            line_index += spacing_width;
        }
    }
    // 在间隔范围内找出1个可用的点
    fn range_point(&self, mut line_index: isize, mut aabb: Aabb, flag: u8, vec: &mut Vec<Point>) {
        if aabb.max.x > self.width as isize {
            aabb.max.x = self.width as isize;
        }
        if aabb.max.y > self.height as isize {
            aabb.max.y = self.height as isize;
        }
        for y in aabb.min.y..aabb.max.y {
            for x in aabb.min.x..aabb.max.x {
                if !self.is_node_center_obstacle(NodeIndex((x + line_index) as usize), flag) {
                    vec.push(Point::new(x, y));
                    return;
                }
            }
            line_index += self.width as isize;
        }
    }
}

// center_flag要求最大0b1111, side_flag要求最大0b11
pub struct FlagTileMap<'a> {
    map: &'a TileMap,
    center_flag: u8,
    side_flag: u8,
}
impl<'a> Map<usize> for FlagTileMap<'a> {
    type NodeIter = NodeIterator;

    fn get_neighbors(&self, cur: NodeIndex, parent: NodeIndex) -> Self::NodeIter {
        let mut arr = get_8d_neighbors(cur.0, self.map.width, self.map.amount);
        let (_, r, d) = self.get_obstacle(cur.0);
        // 检查当前点的右边和下边
        if r {
            arr[Direction::UpRight as usize] = Null::null();
            arr[Direction::Right as usize] = Null::null();
            arr[Direction::DownRight as usize] = Null::null();
        }
        if d {
            arr[Direction::DownLeft as usize] = Null::null();
            arr[Direction::Down as usize] = Null::null();
            arr[Direction::DownRight as usize] = Null::null();
        }

        // 处理右边是否可达
        let i = arr[Direction::Right as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, _, down) = self.get_obstacle(i);
                if ok {
                    arr[Direction::UpRight as usize] = Null::null();
                    arr[Direction::Right as usize] = Null::null();
                    arr[Direction::DownRight as usize] = Null::null();
                } else if down {
                    arr[Direction::DownRight as usize] = Null::null();
                }
            } else {
                arr[Direction::UpRight as usize] = Null::null();
                arr[Direction::Right as usize] = Null::null();
                arr[Direction::DownRight as usize] = Null::null();
            }
        }
        // 处理左边是否可达
        let i = arr[Direction::Left as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, down) = self.get_obstacle(i);
                if ok || right {
                    arr[Direction::UpLeft as usize] = Null::null();
                    arr[Direction::Left as usize] = Null::null();
                    arr[Direction::DownLeft as usize] = Null::null();
                } else if down {
                    arr[Direction::DownLeft as usize] = Null::null();
                }
            } else {
                arr[Direction::UpLeft as usize] = Null::null();
                arr[Direction::Left as usize] = Null::null();
                arr[Direction::DownLeft as usize] = Null::null();
            }
        }
        // 处理下边是否可达
        let i = arr[Direction::Down as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, _) = self.get_obstacle(i);
                if ok {
                    arr[Direction::DownLeft as usize] = Null::null();
                    arr[Direction::Down as usize] = Null::null();
                    arr[Direction::DownRight as usize] = Null::null();
                } else if right {
                    arr[Direction::DownRight as usize] = Null::null();
                }
            } else {
                arr[Direction::DownLeft as usize] = Null::null();
                arr[Direction::Down as usize] = Null::null();
                arr[Direction::DownRight as usize] = Null::null();
            }
        }

        // 处理上边是否可达
        let i = arr[Direction::Up as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, down) = self.get_obstacle(i);
                if ok || down {
                    arr[Direction::UpLeft as usize] = Null::null();
                    arr[Direction::Up as usize] = Null::null();
                    arr[Direction::UpRight as usize] = Null::null();
                } else if right {
                    arr[Direction::UpRight as usize] = Null::null();
                }
            } else {
                arr[Direction::UpLeft as usize] = Null::null();
                arr[Direction::Up as usize] = Null::null();
                arr[Direction::UpRight as usize] = Null::null();
            }
        }
        // 处理左上是否可达
        let i = arr[Direction::UpLeft as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, down) = self.get_obstacle(i);
                if ok || right || down {
                    arr[Direction::UpLeft as usize] = Null::null();
                }
            } else {
                arr[Direction::UpLeft as usize] = Null::null();
            }
        }
        // 处理右上是否可达
        let i = arr[Direction::UpRight as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, _, down) = self.get_obstacle(i);
                if ok || down {
                    arr[Direction::UpRight as usize] = Null::null();
                }
            } else {
                arr[Direction::UpRight as usize] = Null::null();
            }
        }
        // 处理左下是否可达
        let i = arr[Direction::DownLeft as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, right, _) = self.get_obstacle(i);
                if ok || right {
                    arr[Direction::DownLeft as usize] = Null::null();
                }
            } else {
                arr[Direction::DownLeft as usize] = Null::null();
            }
        }
        // 处理右下是否可达
        let i = arr[Direction::DownRight as usize];
        if !i.is_null() {
            if parent.0 != i {
                let (ok, _, _) = self.get_obstacle(i);
                if ok {
                    arr[Direction::DownRight as usize] = Null::null();
                }
            } else {
                arr[Direction::DownRight as usize] = Null::null();
            }
        }
        NodeIterator { arr, index: 0 }
    }

    fn get_g(&self, cur: NodeIndex, parent: NodeIndex) -> usize {
        if cur.0 == parent.0 + 1
            || cur.0 + 1 == parent.0
            || cur.0 == parent.0 + self.map.width
            || cur.0 + self.map.width == parent.0
        {
            return self.map.cell_len;
        }
        self.map.oblique_len
    }
    fn get_h(&self, cur: NodeIndex, end: NodeIndex) -> usize {
        let from_p = get_xy(self.map.width, cur);
        let to_p = get_xy(self.map.width, end);
        let x = (from_p.x - to_p.x).abs() as usize;
        let y = (from_p.y - to_p.y).abs() as usize;
        let (min, max) = if x <= y { (x, y) } else { (y, x) };

        let t = min;
        let r = max - min;
        // 45度斜线的长度 + 剩余直线的长度
        t * self.map.oblique_len + r * self.map.cell_len
    }
}
impl<'a> FlagTileMap<'a> {
    pub fn new(map: &'a TileMap, center_flag: u8, side_flag: u8) -> Self {
        FlagTileMap {
            map,
            center_flag: center_flag & CENTER_MASK,
            side_flag: side_flag & SIDE_MASK,
        }
    }
    /// 获得指定位置的障碍物描述（中心是否障碍， 右边是否障碍， 下边是否障碍）
    /// 通过中心标识和参数中心标识与（&）来判断，为0表示中心障碍
    /// 通过右标识和参数边标识与（&）来判断，为0表示右边障碍
    /// 通过下标识和参数边标识与（&）来判断，为0表示下边障碍
    /// center_flag要求最大0b1111, side_flag要求最大0b11
    #[inline]
    pub fn get_obstacle(&self, index: usize) -> (bool, bool, bool) {
        let i = unsafe { *self.map.nodes.get_unchecked(index) };
        (
            i & self.center_flag == 0,
            i & (self.side_flag << RIGHT_INDEX) == 0,
            i & (self.side_flag << DOWN_INDEX) == 0,
        )
    }
    // 获得指定范围所有可用的点
    pub fn list(&self, aabb: &Aabb) -> ListIter {
        ListIter {
            map: self,
            width: self.map.width,
            x: aabb.min.x as usize,
            y: aabb.min.y as usize,
            start_x: aabb.min.x as usize,
            end_x: aabb.max.x as usize,
            end_y: aabb.max.y as usize,
            line_index: aabb.min.y as usize * self.map.width,
        }
    }
}
#[derive(Clone)]
pub struct ListIter<'a> {
    map: &'a FlagTileMap<'a>,
    width: usize,
    x: usize,
    y: usize,
    start_x: usize,
    end_x: usize,
    end_y: usize,
    line_index: usize,
}
impl<'a> ListIter<'a> {
    fn step(&mut self) {
        self.x += 1;
        if self.x >= self.end_x {
            self.x = self.start_x;
            self.line_index += self.width;
            self.y += 1;
        }
    }
}
impl<'a> Iterator for ListIter<'a> {
    type Item = Point;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.y >= self.end_y {
                return None;
            }
            let (c, r, d) = self.map.get_obstacle(self.x + self.line_index);
            if c && r && d {
                self.step();
                continue;
            }
            let r = Some(Point::new(self.x as isize, self.y as isize));
            self.step();
            return r;
        }
    }
}

// 遍历邻居的迭代器
#[derive(Debug, Clone)]
pub struct NodeIterator {
    arr: [usize; 8],
    index: usize,
}

impl Iterator for NodeIterator {
    type Item = NodeIndex;
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.index >= self.arr.len() {
                return None;
            }
            let i = self.arr[self.index];
            self.index += 1;
            if !i.is_null() {
                return Some(NodeIndex(i));
            }
        }
    }
}
/// 路径过滤器，合并相同方向的路径点
#[derive(Clone)]
pub struct PathFilterIter<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    result: ResultIterator<'a, N, E>,
    // 地图的列
    pub width: usize,
    pub start: Point,
    pub cur: Point,
    angle: Angle,
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> PathFilterIter<'a, N, E> {
    pub fn new(mut result: ResultIterator<'a, N, E>, width: usize) -> Self {
        let start = result.next().unwrap();
        let start = get_xy(width, start);
        let cur = if let Some(c) = result.next() {
            get_xy(width, c)
        } else {
            Null::null()
        };
        let angle = Angle::new(cur - start);
        PathFilterIter {
            result,
            width,
            start,
            cur,
            angle,
        }
    }
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> Iterator
    for PathFilterIter<'a, N, E>
{
    type Item = Point;
    fn next(&mut self) -> Option<Self::Item> {
        if self.start.is_null() {
            return None;
        }
        if self.cur.is_null() {
            return Some(replace(&mut self.start, Null::null()));
        }
        for node in &mut self.result {
            let p = get_xy(self.width, node);
            let angle = Angle::new(p - self.cur);
            if angle != self.angle {
                self.angle = angle;
                let node = replace(&mut self.cur, p);
                return Some(replace(&mut self.start, node));
            }
            self.cur = p;
        }
        let node = replace(&mut self.cur, Null::null());
        Some(replace(&mut self.start, node))
    }
}

/// 路径平滑器，合并直接可达的路径点，采用佛洛依德路径平滑算法（FLOYD），
/// 判断在地图上两个点间的每个格子是否都可以走，算出直线划过的格子，其实就是画直线的算法，
/// https://zhuanlan.zhihu.com/p/34074528
#[derive(Clone)]
pub struct PathSmoothIter<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> {
    result: PathFilterIter<'a, N, E>,
    map: &'a FlagTileMap<'a>,
    start: Point,
    cur: Point,
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> PathSmoothIter<'a, N, E> {
    pub fn new(mut result: PathFilterIter<'a, N, E>, map: &'a FlagTileMap) -> Self {
        let start = result.next().unwrap();
        let cur = if let Some(c) = result.next() {
            c
        } else {
            Null::null()
        };
        PathSmoothIter {
            result,
            map,
            start,
            cur,
        }
    }
}
impl<'a, N: PartialOrd + Zero + Copy + Debug, E: NodeEntry<N> + Default> Iterator
    for PathSmoothIter<'a, N, E>
{
    type Item = Point;
    fn next(&mut self) -> Option<Self::Item> {
        if self.start.is_null() {
            return None;
        }
        if self.cur.is_null() {
            return Some(replace(&mut self.start, Null::null()));
        }
        for node in &mut self.result {
            // 判断该点和起点是否直线可达
            if test_line(self.map, node, self.start).is_some() {
                let node = replace(&mut self.cur, node);
                return Some(replace(&mut self.start, node));
            }
            self.cur = node;
        }
        let node = replace(&mut self.cur, Null::null());
        Some(replace(&mut self.start, node))
    }
}
/// 判断是否地图上直线可达，返回None表示可达，否则返回最后的可达点
pub fn test_line(map: &FlagTileMap, start: Point, end: Point) -> Option<Point> {
    // println!("test_line, start:{:?} end:{:?}", start, end);
    if start == end {
        return None;
    }
    let b = Bresenham::new(start, end);
    let c = map.map.width as isize;
    // 由于y越大越向下，所以y轴被翻转了
    if end.x > start.x {
        if end.y > start.y {
            // 第1象限，等于正常坐标轴的第4象限
            if b.xy_change {
                // 270-315度
                test_line2(
                    map,
                    b,
                    yx,
                    check_down,
                    check_center_down,
                    check_center_down,
                    check_center_right,
                    check_center_down,
                    check_center,
                    check_center,
                    -1,
                    -c,
                )
            } else {
                // 315-360度
                test_line2(
                    map,
                    b,
                    xy,
                    check_right,
                    check_center_right,
                    check_center_right,
                    check_center_right,
                    check_center_down,
                    check_center,
                    check_center,
                    -1,
                    -c,
                )
            }
        } else {
            // 第2象限，等于正常坐标轴的第1象限
            if b.xy_change {
                // 45-90度
                test_line2(
                    map,
                    b,
                    yx,
                    check,
                    check_center_down,
                    check_center_down,
                    check_center_right,
                    check_center,
                    check_center,
                    check_center_down,
                    -1,
                    c,
                )
            } else {
                // 0-45度
                test_line1(
                    map,
                    b,
                    xy,
                    check_right,
                    check_center_right,
                    check_all,
                    check_all,
                    check_center,
                    check_center,
                    check_center_down,
                    -1,
                    c,
                )
            }
        }
    } else if end.y < start.y {
        // 第3象限，等于正常坐标轴的第2象限
        if b.xy_change {
            // 135-180度
            test_line1(
                map,
                b,
                yx,
                check,
                check_center_down,
                check_all,
                check_center_down,
                check_center_right,
                check_center_down,
                check_all,
                1,
                c,
            )
        } else {
            // 90-135度
            test_line1(
                map,
                b,
                xy,
                check,
                check_center_right,
                check_all,
                check_center_down,
                check_center_right,
                check_center_right,
                check_all,
                1,
                c,
            )
        }
    } else {
        // 第4象限，等于正常坐标轴的第3象限
        if b.xy_change {
            // 225-270度
            test_line1(
                map,
                b,
                yx,
                check_down,
                check_center_down,
                check_all,
                check_center,
                check_all,
                check_center,
                check_center_right,
                1,
                -c,
            )
        } else {
            // 180-225度
            test_line2(
                map,
                b,
                xy,
                check,
                check_center_right,
                check_center_right,
                check_center,
                check_all,
                check_center,
                check_all,
                1,
                -c,
            )
        }
    }
}
#[inline]
fn xy(p: Point) -> Point {
    p
}
#[inline]
fn yx(p: Point) -> Point {
    Point::new(p.y, p.x)
}
#[inline]
fn check(_map: &FlagTileMap, _index: isize) -> bool {
    false
}
fn check_center(map: &FlagTileMap, index: isize) -> bool {
    let (center, _right, _down) = map.get_obstacle(index as usize);
    center
}
#[inline]
fn check_right(map: &FlagTileMap, index: isize) -> bool {
    let (_center, right, _down) = map.get_obstacle(index as usize);
    right
}
fn check_center_right(map: &FlagTileMap, index: isize) -> bool {
    let (center, right, _down) = map.get_obstacle(index as usize);
    center || right
}
#[inline]
fn check_down(map: &FlagTileMap, index: isize) -> bool {
    let (_center, _right, down) = map.get_obstacle(index as usize);
    down
}
#[inline]
fn check_center_down(map: &FlagTileMap, index: isize) -> bool {
    //println!("check_center_down, {:?}", index);
    let (center, _right, down) = map.get_obstacle(index as usize);
    center || down
}
#[inline]
fn check_all(map: &FlagTileMap, index: isize) -> bool {
    let (center, right, down) = map.get_obstacle(index as usize);
    center || right || down
}
#[inline]
fn get_index(p: Point, width: usize) -> isize {
    p.x + p.y * (width as isize)
}
/// 判断是否地图上直线可达
#[inline]
fn test_line1(
    map: &FlagTileMap,
    mut b: Bresenham,
    change: fn(Point) -> Point,
    start_check: fn(&FlagTileMap, isize) -> bool,
    check_line: fn(&FlagTileMap, isize) -> bool,
    check_oblique: fn(&FlagTileMap, isize) -> bool,
    check_oblique1: fn(&FlagTileMap, isize) -> bool,
    check_oblique2: fn(&FlagTileMap, isize) -> bool,
    check_line_end: fn(&FlagTileMap, isize) -> bool,
    check_oblique_end: fn(&FlagTileMap, isize) -> bool,
    oblique1: isize,
    oblique2: isize,
) -> Option<Point> {
    let index = get_index(change(b.start), map.map.width);
    if start_check(map, index) {
        return Some(b.start);
    }
    let mut last = b.start;
    b.step();
    while b.start.x != b.end.x || b.start.y != b.end.y {
        // 如果和终点直线连通，则可判断为直线可达
        let index = get_index(change(b.start), map.map.width);
        if b.start.y != last.y {
            // 为斜线
            if check_oblique(map, index)
                || check_oblique1(map, index + oblique1)
                || check_oblique2(map, index + oblique2)
            {
                return Some(change(last));
            }
            if check_oblique(map, index) {
                return Some(change(last));
            }
        } else {
            if check_line(map, index) {
                return Some(change(last));
            }
        }
        last = b.start;
        b.step();
    }
    let index = get_index(change(b.start), map.map.width);
    if b.start.y != last.y {
        // 为斜线
        if check_oblique_end(map, index)
            || check_oblique1(map, index + oblique1)
            || check_oblique2(map, index + oblique2)
        {
            return Some(change(last));
        }
    } else {
        if check_line_end(map, index) {
            return Some(change(last));
        }
    }
    None
}
/// 判断是否地图上直线可达
#[inline]
fn test_line2(
    map: &FlagTileMap,
    mut b: Bresenham,
    change: fn(Point) -> Point,
    start_check: fn(&FlagTileMap, isize) -> bool,
    check_line: fn(&FlagTileMap, isize) -> bool,
    check_oblique: fn(&FlagTileMap, isize) -> bool,
    check_oblique1: fn(&FlagTileMap, isize) -> bool,
    check_oblique2: fn(&FlagTileMap, isize) -> bool,
    check_line_end: fn(&FlagTileMap, isize) -> bool,
    check_oblique_end: fn(&FlagTileMap, isize) -> bool,
    oblique1: isize,
    oblique2: isize,
) -> Option<Point> {
    let index = get_index(change(b.start), map.map.width);
    if start_check(map, index) {
        return Some(b.start);
    }
    let mut last = b.start;
    b.step();
    while b.start.x != b.end.x || b.start.y != b.end.y {
        // 如果和终点直线连通，则可判断为直线可达
        let index = get_index(change(b.start), map.map.width);
        if b.start.y != last.y {
            // 为斜线
            if check_oblique(map, index)
                || check_oblique1(map, index + oblique1)
                || check_oblique2(map, index + oblique2)
            {
                return Some(change(last));
            }
            if check_oblique(map, index) {
                return Some(change(last));
            }
        } else if b.y(b.start.x + b.step) != last.y {
            // 下次是斜线
            if check_all(map, index) {
                return Some(change(last));
            }
        } else {
            // 下次也不是斜线
            //println!("test_line2, {:?}", index);
            if check_line(map, index) {
                return Some(change(last));
            }
        }
        last = b.start;
        b.step();
    }
    let index = get_index(change(b.start), map.map.width);
    if b.start.y != last.y {
        // 为斜线
        if check_oblique_end(map, index)
            || check_oblique1(map, index + oblique1)
            || check_oblique2(map, index + oblique2)
        {
            return Some(change(last));
        }
    } else {
        if check_line_end(map, index) {
            return Some(change(last));
        }
    }
    None
}

/// 获得指定位置瓦片的坐标
pub fn get_xy(width: usize, index: NodeIndex) -> Point {
    Point::new((index.0 % width) as isize, (index.0 / width) as isize)
}

//#![feature(test)]
#[cfg(test)]
mod test_tilemap {
    use std::mem::transmute;

    use crate::{
        base::{Aabb, Point},
        finder::{AStar, NodeIndex},
        normal::{make_neighbors, Entry},
        tile_map::{test_line, FlagTileMap, PathFilterIter, PathSmoothIter, TileMap},
        *,
    };
    //use rand_core::SeedableRng;
    use test::Bencher;

    #[test]
    fn test1() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(10, 10, 100, 141);
        let mut astar: AStar<usize, Entry<usize>> =
            AStar::with_capacity(map.width * map.height, 100);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        map.set_range_flag(&Aabb::new(Point::new(4, 0), Point::new(6, 4)), 0);
        map.set_range_flag(&Aabb::new(Point::new(4, 5), Point::new(6, 10)), 0);
        let mut map = FlagTileMap::new(&map, 1, 1);
        let r = test_line(&map, Point::new(2, 5), Point::new(6, 4));
        assert_eq!(r, Some(Point { x: 3, y: 5 }));

        let start = NodeIndex(9 + 9 * map.map.width);
        let end = NodeIndex(1 + 9 * map.map.width);

        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        println!("r: {:?}", r);

        let mut rr = vec![];
        for r in PathFilterIter::new(astar.result_iter(end), map.map.width) {
            rr.push(r);
        }
        assert_eq!(
            rr,
            vec![
                Point { x: 1, y: 9 },
                Point { x: 1, y: 6 },
                Point { x: 3, y: 4 },
                Point { x: 6, y: 4 },
                Point { x: 9, y: 7 },
                Point { x: 9, y: 9 }
            ]
        );
        rr.clear();
        let f = PathFilterIter::new(astar.result_iter(end), map.map.width);
        for r in PathSmoothIter::new(f, &map) {
            rr.push(r);
        }
        assert_eq!(
            rr,
            vec![
                Point { x: 1, y: 9 },
                Point { x: 3, y: 4 },
                Point { x: 6, y: 4 },
                Point { x: 9, y: 9 }
            ]
        );
        rr.clear();
    }

    #[test]
    fn test2() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(11, 11, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        map.set_node_center_flag(NodeIndex(4 + 4 * 11), 0);
        map.set_node_center_flag(NodeIndex(5 + 4 * 11), 0);
        map.set_node_center_flag(NodeIndex(6 + 4 * 11), 0);

        map.set_node_center_flag(NodeIndex(4 + 5 * 11), 0);
        map.set_node_center_flag(NodeIndex(5 + 5 * 11), 0);
        map.set_node_center_flag(NodeIndex(6 + 5 * 11), 0);

        map.set_node_center_flag(NodeIndex(4 + 6 * 11), 0);
        map.set_node_center_flag(NodeIndex(5 + 6 * 11), 0);
        map.set_node_center_flag(NodeIndex(6 + 6 * 11), 0);

        let mut astar: AStar<usize, Entry<usize>> =
            AStar::with_capacity(map.width * map.height, 100);

        let start = NodeIndex(0);
        let end = NodeIndex(120);
        let mut map = FlagTileMap::new(&map, 1, 1);

        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        println!("r: {:?}", r);
        let mut rr = vec![];

        for r in PathFilterIter::new(astar.result_iter(end), map.map.width) {
            rr.push(r);
        }
        assert_eq!(
            rr,
            vec![
                Point { x: 10, y: 10 },
                Point { x: 9, y: 10 },
                Point { x: 6, y: 7 },
                Point { x: 3, y: 7 },
                Point { x: 3, y: 3 },
                Point { x: 0, y: 0 }
            ]
        );
        rr.clear();
        let f = PathFilterIter::new(astar.result_iter(end), map.map.width);
        for r in PathSmoothIter::new(f, &map) {
            rr.push(r);
        }
        assert_eq!(
            rr,
            vec![
                Point { x: 10, y: 10 },
                Point { x: 3, y: 7 },
                Point { x: 0, y: 0 }
            ]
        );
        rr.clear();
    }
    #[test]
    fn test22() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(11, 11, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );

        map.set_node_center_flag(NodeIndex(4 + 4 * 11), 0);
        map.set_node_center_flag(NodeIndex(5 + 4 * 11), 0);
        map.set_node_center_flag(NodeIndex(6 + 4 * 11), 0);

        map.set_node_center_flag(NodeIndex(4 + 5 * 11), 0);
        map.set_node_center_flag(NodeIndex(5 + 5 * 11), 0);
        map.set_node_center_flag(NodeIndex(6 + 5 * 11), 0);

        map.set_node_center_flag(NodeIndex(4 + 6 * 11), 0);
        map.set_node_center_flag(NodeIndex(5 + 6 * 11), 0);
        map.set_node_center_flag(NodeIndex(6 + 6 * 11), 0);

        let mut astar: AStar<usize, Entry<usize>> =
            AStar::with_capacity(map.width * map.height, 100);

        let start = NodeIndex(120);
        let end = NodeIndex(0);
        let mut map = FlagTileMap::new(&map, 1, 1);

        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        println!("r: {:?}", r);
        let mut rr = vec![];

        for r in PathFilterIter::new(astar.result_iter(end), map.map.width) {
            rr.push(r);
        }
        // println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 0, y: 0 },
                Point { x: 1, y: 0 },
                Point { x: 4, y: 3 },
                Point { x: 7, y: 3 },
                Point { x: 7, y: 7 },
                Point { x: 10, y: 10 }
            ]
        );
        rr.clear();
        let f = PathFilterIter::new(astar.result_iter(end), map.map.width);
        for r in PathSmoothIter::new(f, &map) {
            rr.push(r);
        }
        // println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 0, y: 0 },
                Point { x: 7, y: 3 },
                Point { x: 10, y: 10 }
            ]
        );
        rr.clear();
    }
    #[test]
    fn test3() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(1000, 1000, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        map.set_node_center_flag(NodeIndex(88 + 88 * map.width), 0);
        map.set_node_center_flag(NodeIndex(65 + 64 * map.width), 0);
        map.set_node_center_flag(NodeIndex(54 + 55 * map.width), 0);
        map.set_node_center_flag(NodeIndex(44 + 44 * map.width), 0);
        map.set_node_center_flag(NodeIndex(33 + 33 * map.width), 0);

        let x1 = 999; //rng.next_u32() as usize%map.width;
        let y1 = 999; //rng.next_u32()as usize /map.width;
        let x2 = 1; //rng.next_u32() as usize%map.width;
        let y2 = 1; //rng.next_u32()as usize /map.width;
        println!("x1:{},y1:{}, x2:{},y2:{}", x1, y1, x2, y2);
        let mut astar: AStar<usize, Entry<usize>> =
            AStar::with_capacity(map.width * map.height, 100);
        let start = NodeIndex(x1 + y1 * map.width);
        let end = NodeIndex(x2 + y2 * map.width);
        let mut map = FlagTileMap::new(&map, 1, 1);
        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        let mut rr = vec![];
        for r in PathFilterIter::new(astar.result_iter(end), map.map.width) {
            rr.push(r);
        }
        assert_eq!(
            rr,
            vec![
                Point { x: 1, y: 1 },
                Point { x: 1, y: 4 },
                Point { x: 53, y: 56 },
                Point { x: 54, y: 56 },
                Point { x: 87, y: 89 },
                Point { x: 89, y: 89 },
                Point { x: 999, y: 999 }
            ]
        );
        let f = PathFilterIter::new(astar.result_iter(end), map.map.width);
        rr.clear();
        for r in PathSmoothIter::new(f, &map) {
            rr.push(r);
        }
        assert_eq!(
            rr,
            vec![
                Point { x: 1, y: 1 },
                Point { x: 53, y: 56 },
                Point { x: 87, y: 89 },
                Point { x: 999, y: 999 }
            ]
        );
        rr.clear();
    }
    #[test]
    fn test4() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(30, 30, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );

        map.set_node_center_flag(NodeIndex(13 + 10 * 30), 0);
        map.set_node_center_flag(NodeIndex(14 + 10 * 30), 0);

        let mut astar: AStar<usize, Entry<usize>> =
            AStar::with_capacity(map.width * map.height, 1000);

        let start = NodeIndex(24 + 10 * 30);
        let end = NodeIndex(10 + 10 * 30);
        let mut map = FlagTileMap::new(&map, 1, 1);

        let r = astar.find(start, end, 30000, &mut map, make_neighbors);
        println!("r: {:?}", r);
        let mut rr = vec![];

        for r in PathFilterIter::new(astar.result_iter(end), map.map.width) {
            rr.push(r);
        }
        //println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 10, y: 10 },
                Point { x: 11, y: 11 },
                Point { x: 15, y: 11 },
                Point { x: 16, y: 10 },
                Point { x: 24, y: 10 }
            ]
        );
        rr.clear();
        let f = PathFilterIter::new(astar.result_iter(end), map.map.width);
        for r in PathSmoothIter::new(f, &map) {
            rr.push(r);
        }
        //println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 10, y: 10 },
                Point { x: 11, y: 11 },
                Point { x: 15, y: 11 },
                Point { x: 24, y: 10 }
            ]
        );
        rr.clear();
        let r = map.map.find_round(
            NodeIndex(20 + 4 * map.map.width),
            7,
            0,
            unsafe { transmute(0) },
            1,
            &mut rr,
        );
        //println!("r: {:?} rr: {:?}", r, rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 19, y: 4 },
                Point { x: 20, y: 4 },
                Point { x: 19, y: 5 },
                Point { x: 20, y: 5 },
                Point { x: 21, y: 4 },
                Point { x: 21, y: 5 },
                Point { x: 19, y: 3 },
                Point { x: 20, y: 3 },
                Point { x: 21, y: 3 }
            ]
        );
        rr.clear();
    }
    #[test]
    fn test5() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(11, 11, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        map.set_node_center_flag(NodeIndex(48), 0);
        map.set_node_center_flag(NodeIndex(49), 0);
        map.set_node_center_flag(NodeIndex(50), 0);
        map.set_node_center_flag(NodeIndex(59), 0);
        map.set_node_center_flag(NodeIndex(60), 0);
        map.set_node_center_flag(NodeIndex(61), 0);
        map.set_node_center_flag(NodeIndex(70), 0);
        map.set_node_center_flag(NodeIndex(71), 0);
        map.set_node_center_flag(NodeIndex(72), 0);
        map.set_node_center_flag(NodeIndex(72), 1);
        map.set_node_center_flag(NodeIndex(73), 0);

        assert_eq!(map.get_node_center_flag(NodeIndex(49)), 0);
        assert_eq!(map.get_node_center_flag(NodeIndex(4)), 15);
        assert_eq!(map.get_node_center_flag(NodeIndex(72)), 1);
        assert_eq!(map.get_node_center_flag(NodeIndex(73)), 0);
        let mut rr = vec![];
        let r = map.find_round(NodeIndex(0), 2, 0, unsafe { transmute(1) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(rr, vec![Point { x: 0, y: 0 }, Point { x: 1, y: 0 }]);
        rr.clear();
        let r = map.find_round(NodeIndex(0), 2, 0, unsafe { transmute(2) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(rr, vec![Point { x: 0, y: 0 }, Point { x: 0, y: 1 }]);
        rr.clear();

        let r = map.find_round(NodeIndex(0), 3, 0, unsafe { transmute(0) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 0, y: 0 },
                Point { x: 1, y: 0 },
                Point { x: 0, y: 1 },
                Point { x: 1, y: 1 }
            ]
        );
        rr.clear();
        let r = map.find_round(NodeIndex(0), 5, 1, unsafe { transmute(0) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 0, y: 0 },
                Point { x: 2, y: 0 },
                Point { x: 0, y: 2 },
                Point { x: 2, y: 2 },
                Point { x: 4, y: 0 },
                Point { x: 4, y: 2 }
            ]
        );
        rr.clear();
        let r = map.find_round(NodeIndex(47), 2, 0, unsafe { transmute(0) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(rr, vec![Point { x: 3, y: 4 }, Point { x: 2, y: 4 }]);
        rr.clear();
        let r = map.find_round(NodeIndex(47), 2, 1, unsafe { transmute(0) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(rr, vec![Point { x: 3, y: 4 }, Point { x: 1, y: 4 }]);
        rr.clear();
        let r = map.find_round(NodeIndex(47), 5, 1, unsafe { transmute(0) }, 1, &mut rr);
        //println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 1, y: 4 },
                Point { x: 3, y: 4 },
                Point { x: 1, y: 6 },
                Point { x: 3, y: 6 },
                Point { x: 6, y: 6 }
            ]
        );
        rr.clear();
        let r = map.find_round(NodeIndex(47), 9, 1, unsafe { transmute(0) }, 1, &mut rr);
        println!("rounds:0, {:?}", r);
        //println!("rr: {:?}", rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 0, y: 1 },
                Point { x: 2, y: 1 },
                Point { x: 4, y: 1 },
                Point { x: 0, y: 3 },
                Point { x: 2, y: 3 },
                Point { x: 4, y: 3 },
                Point { x: 0, y: 5 },
                Point { x: 2, y: 5 },
                Point { x: 6, y: 1 },
                Point { x: 6, y: 3 },
                Point { x: 7, y: 5 }
            ]
        );
        rr.clear();
    }
    #[test]
    fn test6() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(30, 30, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        let mut rr = vec![];
        let r = map.find_round(
            NodeIndex(15 + 24 * 30),
            5,
            0,
            unsafe { transmute(3) },
            1,
            &mut rr,
        );
        //println!("r, {:?}, rr: {:?}", r, rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 14, y: 23 },
                Point { x: 15, y: 23 },
                Point { x: 14, y: 24 },
                Point { x: 15, y: 24 },
                Point { x: 14, y: 25 },
                Point { x: 15, y: 25 }
            ]
        );
        rr.clear();
    }
    #[test]
    fn test7() {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(20, 20, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        map.set_range_flag(
            &Aabb::new(
                Point::new(14, 14),
                Point::new(map.width as isize, map.height as isize),
            ),
            1,
        );
        // map.set_range_obstacle(18..20, 0..6, 0);
        // map.set_range_obstacle(20..22, 0..4, 0);
        let mut rr = vec![];
        let r = map.find_round(
            NodeIndex(15 + 15 * map.width),
            3,
            1,
            unsafe { transmute(0) },
            1,
            &mut rr,
        );
        // println!("r, {:?}, rr: {:?}", r, rr);
        assert_eq!(
            rr,
            vec![
                Point { x: 15, y: 15 },
                Point { x: 13, y: 15 },
                Point { x: 13, y: 17 },
                Point { x: 15, y: 17 }
            ]
        );
        rr.clear();
    }
    #[bench]
    fn bench_test1(b: &mut Bencher) {
        //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
        let mut map = TileMap::new(100, 100, 100, 141);
        map.set_range_flag(
            &Aabb::new(
                Point::new(0, 0),
                Point::new(map.width as isize, map.height as isize),
            ),
            u8::MAX,
        );
        map.set_node_center_flag(NodeIndex(88 + 88 * map.width), 0);
        map.set_node_center_flag(NodeIndex(65 + 64 * map.width), 0);
        map.set_node_center_flag(NodeIndex(54 + 55 * map.width), 0);
        map.set_node_center_flag(NodeIndex(44 + 44 * map.width), 0);
        map.set_node_center_flag(NodeIndex(33 + 33 * map.width), 0);

        let x1 = 99; //rng.next_u32() as usize%map.column;
        let y1 = 99; //rng.next_u32()as usize %map.row;
        let x2 = 1; //rng.next_u32() as usize%map.column;
        let y2 = 1; //rng.next_u32()as usize %map.row;
                    //println!("x1:{},y1:{}, x2:{},y2:{}", x1, y1, x2, y2);
        let mut astar: AStar<usize, Entry<usize>> =
            AStar::with_capacity(map.width * map.height, 100);
        let mut map = FlagTileMap::new(&map, 1, 1);
        b.iter(move || {
            let start = NodeIndex(x1 + y1 * map.map.width);
            let end = NodeIndex(x2 + y2 * map.map.width);
            let _r = astar.find(start, end, 30000, &mut map, make_neighbors);
            let mut vec = Vec::with_capacity(100);
            let f = PathFilterIter::new(astar.result_iter(end), map.map.width);

            for r in PathSmoothIter::new(f, &map) {
                vec.push(r);
                // println!("x:{},y:{}", r.0%map.column, r.0/map.column);
            }
        });
    }
}

#[test]
fn test10() {
    //let mut rng = pcg_rand::Pcg32::seed_from_u64(1238);
    let mut map = TileMap::new(11, 11, 100, 141);
    map.set_range_flag(
        &Aabb::new(
            Point::new(0, 0),
            Point::new(map.width as isize, map.height as isize),
        ),
        u8::MAX,
    );

    map.set_node_center_flag(NodeIndex(0 + 5 * 11), 0);
    map.set_node_center_flag(NodeIndex(1 + 5 * 11), 0);
    map.set_node_center_flag(NodeIndex(2 + 5 * 11), 0);
    map.set_node_center_flag(NodeIndex(3 + 5 * 11), 0);
    map.set_node_center_flag(NodeIndex(4 + 5 * 11), 0);
    map.set_node_center_flag(NodeIndex(5 + 5 * 11), 0);

    map.set_node_center_flag(NodeIndex(5 + 0 * 11), 0);
    map.set_node_center_flag(NodeIndex(5 + 1 * 11), 0);
    map.set_node_center_flag(NodeIndex(5 + 2 * 11), 0);
    map.set_node_center_flag(NodeIndex(5 + 3 * 11), 0);
    map.set_node_center_flag(NodeIndex(5 + 4 * 11), 0);

    let mut astar: AStar<usize, Entry<usize>> = AStar::with_capacity(map.width * map.height, 100);

    let start = NodeIndex(110);
    let end = NodeIndex(15);
    let mut map = FlagTileMap::new(&map, 1, 1);

    let r: AStarResult = astar.find(start, end, 30000, &mut map, make_neighbors);
    println!("r: {:?}", r);
    let r = match r {
        AStarResult::Found => end,
        AStarResult::NotFound(i) => i,
        AStarResult::LimitNotFound(i) => i,
    };

    let mut rr = vec![];

    for r in PathFilterIter::new(astar.result_iter(r), map.map.width) {
        rr.push(r);
    }
    println!("========= rr: {:?}", rr);
}
