// #![cfg(target_arch = "wasm32")]

use crate::{
    base::Point,
    finder::AStarResult,
    finder::{AStar as AStarInner, NodeIndex as NodeIndexInner},
    normal::{make_neighbors, Entry},
    tile_map::PathSmoothIter,
    tile_map::{PathFilterIter, TileMap as TileMapInner},
    mipmap::{MipMap as MipMapInner},
};
use pi_orca::vector2::Vector2;
use std::iter::Rev;
use std::vec::IntoIter;
use wasm_bindgen::prelude::wasm_bindgen;

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
#[derive(Debug, Clone)]
pub struct NodeIndex {
    inner: NodeIndexInner,
}

#[wasm_bindgen]
impl NodeIndex {
    pub fn new(index: usize) -> Self {
        Self {
            inner: NodeIndexInner(index),
        }
    }

    pub fn index(&self) -> usize {
        self.inner.0
    }
}

#[wasm_bindgen]
pub struct TileMap {
    inner: TileMapInner,
}

#[wasm_bindgen]
pub struct ResultPath {
    res: Rev<IntoIter<Vector2>>,
}

#[wasm_bindgen]
impl ResultPath {
    pub fn next(&mut self) -> Option<Vector2> {
        self.res.next()
    }
}

#[wasm_bindgen]
impl TileMap {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            inner: TileMapInner::new(width, height, 100, 144),
        }
    }

    pub fn set_obstacle(&mut self, index: &NodeIndex, obstacle: TileObstacle) {
        let pos_type = match obstacle {
            TileObstacle::Right => 1,
            TileObstacle::Down => 2,
            TileObstacle::Center => 4,
            TileObstacle::None => 0,
        };

        self.inner.set_node_obstacle(index.inner, pos_type);
    }
}

#[wasm_bindgen]
pub struct AStar {
    inner: AStarInner<usize, Entry<usize>>,
}

#[wasm_bindgen]
impl AStar {
    pub fn new(width: usize, height: usize, node_number: usize) -> Self {
        Self {
            inner: AStarInner::with_capacity(width * height, node_number),
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
        start: &NodeIndex,
        end: &NodeIndex,
    ) -> Option<NodeIndex> {
        let result = self.inner.find(
            start.inner,
            end.inner,
            max_number,
            &mut tile_map.inner,
            make_neighbors,
        );

        match result {
            AStarResult::Found => return Some(end.clone()),
            AStarResult::NotFound => return None,
            AStarResult::LimitNotFound(index) => return Some(NodeIndex { inner: index }),
        };
    }

    /*
     * @brief: 输出路径
     * @param[in] node: 从路径的哪一个节点开始输出，一般情况下是终点
     * @param[in] tile_map: 地图
     * #return[in]: 路径迭代器
     */
    pub fn result(&self, node: &NodeIndex, tile_map: &TileMap) -> ResultPath {
        let mut res = vec![];
        for item in PathSmoothIter::new(
            PathFilterIter::new(self.inner.result_iter(node.inner), tile_map.inner.width),
            &tile_map.inner,
        ) {
            res.push(Vector2 {
                x: item.x as f32,
                y: item.y as f32,
            });
        }
        let r = res.into_iter().rev();

        ResultPath { res: r }
    }
}

/*
 * 判断两点之间是否可以直达
 */
#[wasm_bindgen]
pub fn test_line(map: &TileMap, start: Vector2, end: Vector2) -> Option<Vector2> {
    if let Some(pos) = crate::tile_map::test_line(
        &map.inner,
        Point::new(start.x() as isize, start.y() as isize),
        Point::new(end.x() as isize, end.y() as isize),
    ) {
        return Some(Vector2::new(pos.x as f32, pos.y as f32));
    }
    None
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
