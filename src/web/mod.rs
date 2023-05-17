//#![cfg(target_arch = "wasm32")]

use crate::{
    base::Point,
    finder::AStarResult,
    finder::NodeIndex,
    normal::{make_neighbors, Entry},
    tile_map::{sort_by_dist, PathFilterIter, PathSmoothIter},
};
use bytemuck::NoUninit;
use serde_wasm_bindgen::to_value;
use std::mem::transmute;
use wasm_bindgen::prelude::*;

// 瓦片内的障碍
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub enum TileObstacle {
    Right = 1,
    Down = 2,
    Center = 4,
    None = 0,
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
    pub fn set_obstacle(&mut self, index: usize, obstacle: TileObstacle) {
        self.inner
            .set_node_obstacle(NodeIndex(index), unsafe { transmute(obstacle) });
    }
    pub fn get_obstacle(&mut self, index: usize) -> u8 {
        self.inner.get_node_obstacle(NodeIndex(index))
    }
    // 获得指定点周围所有可用的点，周围是顺时针一圈一圈扩大，直到可用点数超过count, spacing为可用点的间隔
    // 参数d: Direction为默认扩展的朝向，0-3都可以设置
    pub fn find_round(&mut self, index: usize, count: usize, spacing: usize, d: Direction)-> JsValue {
        let dd: u8 = unsafe { transmute(d) };
        self.result.clear();
        let aabb = self.inner.find_round(
            NodeIndex(index),
            count,
            spacing,
            unsafe { transmute(dd as u32) },
            &mut self.result,
        );
        if self.result.len() == 0 {
            return to_value(&aabb).unwrap()
        }
        let vec: &Vec<P> = unsafe { transmute(&self.result) };
        let rr: &[i32] = bytemuck::cast_slice(&vec.as_slice());
        round_result(rr);
        return to_value(&aabb).unwrap()
    }
    // 寻找一个点周围可以放置的位置列表，并且按到target的距离进行排序（小-大）
    pub fn find_round_and_sort_by_dist(
        &mut self,
        index: usize,
        count: usize,
        spacing: usize,
        d: Direction,
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
            &mut self.result,
        );
        if self.result.len() == 0 {
            return to_value(&aabb).unwrap()
        }
        sort_by_dist(Point::new(target_x, target_y), &mut self.result);
        let vec: &Vec<P> = unsafe { transmute(&self.result) };
        let rr: &[i32] = bytemuck::cast_slice(&vec.as_slice());
        round_result(rr);
        return to_value(&aabb).unwrap()
    }

    pub fn test_line(&self, start_x: isize, start_y: isize, end_x: isize, end_y: isize) -> JsValue {
        let r = crate::tile_map::test_line(
            &self.inner,
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
    ) -> Option<usize> {
        let result = self.inner.find(
            NodeIndex(start),
            NodeIndex(end),
            max_number,
            &mut tile_map.inner,
            make_neighbors,
        );
        match result {
            AStarResult::Found => return Some(end.clone()),
            AStarResult::NotFound => return None,
            AStarResult::LimitNotFound(index) => return Some(index.0),
        };
    }

    /*
     * @brief: 输出路径
     * @param[in] node: 从路径的哪一个节点开始输出，一般情况下是终点
     * @param[in] tile_map: 地图
     * #return[in]: 路径数组
     */
    pub fn result(&mut self, node: usize, tile_map: &mut TileMap) {
        tile_map.result.clear();
        for r in PathSmoothIter::new(
            PathFilterIter::new(
                self.inner.result_iter(NodeIndex(node)),
                tile_map.inner.width,
            ),
            &tile_map.inner,
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
