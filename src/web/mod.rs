// #![cfg(target_arch = "wasm32")]

use crate::{
    base::Point,
    finder::AStarResult,
    finder::{NodeIndex},
    normal::{make_neighbors, Entry},
    tile_map::{PathSmoothIter, PathFilterIter}, mipmap::sort_by_dist,
};
use std::{mem::transmute, num::NonZeroI32};
use bytemuck::NoUninit;
use wasm_bindgen::prelude::*;
use serde_wasm_bindgen::to_value;
use js_sys::{ArrayBuffer, Uint32Array, Function, Int32Array};


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
}

#[wasm_bindgen]
impl TileMap {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            inner: crate::tile_map::TileMap::new(width, height, 100, 144),
        }
    }
    pub fn set_obstacle(&mut self, index: usize, obstacle: TileObstacle) {
        self.inner.set_node_obstacle(NodeIndex(index), unsafe{transmute(obstacle)});
    }
    pub fn get_obstacle(&mut self, index: usize) -> u8{
        self.inner.get_node_obstacle(NodeIndex(index))
    }
}

// 四方向枚举
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub enum Location {
    UpLeft = 0,
    UpRight = 1,
    DownLeft = 2,
    DownRight = 3,
}

#[wasm_bindgen]
pub struct MipMap {
    inner: crate::mipmap::MipMap,
    result: Vec<Point>,
}

#[wasm_bindgen]
impl MipMap {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            inner: crate::mipmap::MipMap::new(width, height),
            result: Default::default(),
        }
    }
    pub fn is_true(&mut self, index: usize) -> bool {
        self.inner.is_true(NodeIndex(index))
    }
    pub fn set_true(&mut self, index: usize) -> bool {
        self.inner.set_true(NodeIndex(index))
    }
    pub fn set_false(&mut self, index: usize) -> bool {
        self.inner.set_false(NodeIndex(index))
    }
    pub fn move_to(&mut self, src: usize, dest: usize) -> bool {
        self.inner.move_to(NodeIndex(src), NodeIndex(dest))
    }
    pub fn find_round(&self, index: usize, d: Location, count: usize) -> JsValue {
        let dd: u8 = unsafe{transmute(d)};
        let r = self.inner.find_round(NodeIndex(index), unsafe{transmute(dd as u32)}, count);
        to_value(&r).unwrap()
    }
    pub fn find_round_list_sort_by_dist(&mut self, index: usize, d: Location, count: usize, d_x: isize, d_y: isize) -> Int32Array {
        let dd: u8 = unsafe{transmute(d)};
        let r = self.inner.find_round(NodeIndex(index), unsafe{transmute(dd as u32)}, count);
        if r.1 == 0 {
            return Int32Array::new(&JsValue::from_f64(0.0))
        }
        self.result.clear();
        for p in self.inner.list(r.0) {
            self.result.push(p);
        }
        sort_by_dist(Point::new(d_x, d_y), &mut self.result);
        let vec: &Vec<P> = unsafe{transmute(&self.result)};
        let rr: &[i32] = bytemuck::cast_slice(&vec.as_slice());
        let arr = Int32Array::new(&JsValue::from_f64(rr.len() as f64));
        arr.copy_from(rr);
        arr
    }
}

#[derive(Clone, Copy)]
struct P(i32, i32);

unsafe impl NoUninit for P {}

#[wasm_bindgen]
pub struct AStar {
    inner: crate::finder::AStar<usize, Entry<usize>>,
    result: Vec<u32>,
}

#[wasm_bindgen]
impl AStar {
    pub fn new(width: usize, height: usize, node_number: usize) -> Self {
        Self {
            inner: crate::finder::AStar::with_capacity(width * height, node_number),
            result: Default::default(),
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
    # [wasm_bindgen (method , js_name = bufferData)]
    pub fn result(&mut self, node: usize, tile_map: &TileMap) {
        self.result.clear();
        for r in PathSmoothIter::new(
            PathFilterIter::new(self.inner.result_iter(NodeIndex(node)), tile_map.inner.width),
            &tile_map.inner,
        ) {
            self.result.push(r.x as u32);
            self.result.push(r.y as u32);
        }
        path_result(self.result.as_slice())
    }
}

/*
 * 判断两点之间是否可以直达
 */
#[wasm_bindgen]
pub fn test_line(map: &TileMap, start_x: isize, start_y: isize, end_x: isize, end_y: isize) -> JsValue {
    let r = crate::tile_map::test_line(
        &map.inner,
        Point::new(start_x, start_y),
        Point::new(end_x, end_y),
    );
    to_value(&r).unwrap()
}

#[wasm_bindgen(module = "/src/web/path_result.js")]
extern "C" {
    fn path_result(arr: &[u32]);
}

#[wasm_bindgen]
pub fn get_obstacle(nodes: &[u8], index: usize) -> Vec<u8> {
    let r = crate::tile_map::get_obstacle(&nodes.to_vec(), index);
    let a = if r.0 { 1 } else { 0 };
    let b = if r.1 { 1 } else { 0 };
    let c = if r.2 { 1 } else { 0 };
    // [r.0, r.1, r.2]
    vec![a, b, c]
}
