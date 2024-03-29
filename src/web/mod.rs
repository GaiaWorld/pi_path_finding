//#![cfg(target_arch = "wasm32")]

use crate::{
    base::{Point, Aabb},
    finder::AStarResult,
    finder::NodeIndex,
    normal::{make_neighbors, Entry},
    tile_map::{sort_by_dist, FlagTileMap, PathFilterIter, PathSmoothIter},
};
use bytemuck::NoUninit;
use serde_wasm_bindgen::to_value;
use std::mem::transmute;
use wasm_bindgen::prelude::*;

// 瓦片标识类型
#[wasm_bindgen]
#[derive(Debug, Clone, PartialEq)]
pub enum TileFlagType {
    All = 0,
    Center = 1,
    Right = 2,
    Down = 4,
}

#[wasm_bindgen]
pub struct TileMap {
    inner: crate::tile_map::TileMap,
    result: Vec<Point>,
}

#[wasm_bindgen]
impl TileMap {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            inner: crate::tile_map::TileMap::new(width, height, 100, 144),
            result: Default::default(),
        }
    }
    pub fn set_node_flag(&mut self, index: usize, flag_type: TileFlagType, value: u8) -> u8 {
        if flag_type == TileFlagType::All {
            self.inner.set_node_flag(NodeIndex(index), value)
        } else {
            self.inner
                .set_node_flag_type(NodeIndex(index), unsafe { transmute(flag_type) }, value)
        }
    }
    pub fn get_node_flag(&mut self, index: usize, flag_type: TileFlagType) -> u8 {
        if flag_type == TileFlagType::All {
            self.inner.get_node_flag(NodeIndex(index))
        } else {
            self.inner
                .get_node_flag_type(NodeIndex(index), unsafe { transmute(flag_type) })
        }
    }
    pub fn set_range_flag(&mut self, x1: usize, y1: usize, x2: usize, y2: usize, value: u8) {
        let aabb = Aabb::new(
            Point::new(x1 as isize, y1 as isize),
            Point::new(x2 as isize, y2 as isize),
        );
        self.inner
                .set_range_flag(&aabb, value)
    }
    // 获得指定点周围所有可用的点，周围是顺时针一圈一圈扩大，直到可用点数超过count, spacing为可用点的间隔
    // 参数d: Direction为默认扩展的朝向，0-3都可以设置
    pub fn find_round(
        &mut self,
        index: usize,
        count: usize,
        spacing: usize,
        d: Direction,
        flag: u8,
    ) -> JsValue {
        let dd: u8 = unsafe { transmute(d) };
        self.result.clear();
        let aabb = self.inner.find_round(
            NodeIndex(index),
            count,
            spacing,
            unsafe { transmute(dd as u32) },
            flag,
            &mut self.result,
        );
        if self.result.len() == 0 {
            return to_value(&aabb).unwrap();
        }
        let vec: &Vec<P> = unsafe { transmute(&self.result) };
        let rr: &[i32] = bytemuck::cast_slice(&vec.as_slice());
        round_result(rr);
        return to_value(&aabb).unwrap();
    }
    // 寻找一个点周围可以放置的位置列表，并且按到target的距离进行排序（小-大）
    pub fn find_round_and_sort_by_dist(
        &mut self,
        index: usize,
        count: usize,
        spacing: usize,
        d: Direction,
        flag: u8,
        target_x: isize,
        target_y: isize,
    ) -> JsValue {
        let dd: u8 = unsafe { transmute(d) };
        self.result.clear();
        let aabb = self.inner.find_round(
            NodeIndex(index),
            count,
            spacing,
            unsafe { transmute(dd as u32) },
            flag,
            &mut self.result,
        );
        if self.result.len() == 0 {
            return to_value(&aabb).unwrap();
        }
        sort_by_dist(Point::new(target_x, target_y), &mut self.result);
        let vec: &Vec<P> = unsafe { transmute(&self.result) };
        let rr: &[i32] = bytemuck::cast_slice(&vec.as_slice());
        round_result(rr);
        return to_value(&aabb).unwrap();
    }

    pub fn test_line(
        &self,
        start_x: isize,
        start_y: isize,
        end_x: isize,
        end_y: isize,
        center_flag: u8,
        side_flag: u8,
    ) -> JsValue {
        let map = FlagTileMap::new(&self.inner, center_flag, side_flag);
        let r = crate::tile_map::test_line(
            &map,
            Point::new(start_x, start_y),
            Point::new(end_x, end_y),
        );
        to_value(&r).unwrap()
    }
}

// 四方向枚举
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub enum Direction {
    Left = 0,
    Right = 1,
    Up = 2,
    Down = 3,
}
#[derive(Clone, Copy)]
struct P(i32, i32);

unsafe impl NoUninit for P {}

#[wasm_bindgen]
pub struct AStar {
    inner: crate::finder::AStar<usize, Entry<usize>>,
}

#[wasm_bindgen]
impl AStar {
    pub fn new(width: usize, height: usize, node_number: usize) -> Self {
        Self {
            inner: crate::finder::AStar::with_capacity(width * height, node_number),
        }
    }

    /*
    @brief: 求解路径
    @param tile_map: 地图
     */
    pub fn find_path(
        &mut self,
        tile_map: &mut TileMap,
        max_number: usize,
        start: usize,
        end: usize,
        center_flag: u8,
        side_flag: u8,
    ) -> Option<usize> {
        let mut map = FlagTileMap::new(&tile_map.inner, center_flag, side_flag);

        let result = self.inner.find(
            NodeIndex(start),
            NodeIndex(end),
            max_number,
            &mut map,
            make_neighbors,
        );
        match result {
            AStarResult::Found => return Some(end.clone()),
            AStarResult::NotFound(index) => return Some(index.0),
            AStarResult::LimitNotFound(index) => return Some(index.0),
        };
    }

    /*
     * @brief: 输出路径
     * @param[in] node: 从路径的哪一个节点开始输出，一般情况下是终点
     * @param[in] tile_map: 地图
     * #return[in]: 路径数组
     */
    pub fn result(&mut self, node: usize, tile_map: &mut TileMap, center_flag: u8, side_flag: u8) {
        tile_map.result.clear();
        let map = FlagTileMap::new(&tile_map.inner, center_flag, side_flag);

        for r in PathSmoothIter::new(
            PathFilterIter::new(
                self.inner.result_iter(NodeIndex(node)),
                tile_map.inner.width,
            ),
            &map,
        ) {
            tile_map.result.push(r);
        }
        let vec: &Vec<P> = unsafe { transmute(&tile_map.result) };
        let rr: &[i32] = bytemuck::cast_slice(&vec.as_slice());
        path_result(rr)
    }
}

#[wasm_bindgen(module = "/src/web/path_result.js")]
extern "C" {
    fn path_result(arr: &[i32]);
    fn round_result(arr: &[i32]);
}
