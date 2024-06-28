use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::sync::Arc;

#[derive(Default)]
pub struct RoadNetwork {
    // vertex id is an integer (i64)
    // edge is HashMap of the <NodeId, Cost>
    pub nodes: HashSet<i64>,
    pub edges: HashMap<i64, HashMap<i64, u32>>,
}

pub struct SimplifiedWay {
    pub id: i64,
    pub highway_speed_m_per_s: f32,
    pub node_sequence: Vec<i64>,
}

pub fn speed_from_way_kmh(way: &osmpbfreader::objects::Way) -> Option<u32> {
    let tags = way.tags.clone();
    let highway = tags
        .into_inner()
        .into_iter()
        .find(|(key, _)| key == &"highway");

    match highway {
        Some(highway) => match highway.1.as_str() {
            "motorway" => Some(110),
            "trunk" => Some(110),
            "primary" => Some(70),
            "secondary" => Some(60),
            "tertiary" => Some(50),
            "motorway_link" => Some(50),
            "trunk_link" => Some(50),
            "primary_link" => Some(50),
            "secondary_link" => Some(50),
            "road" => Some(40),
            "unclassified" => Some(40),
            "residential" => Some(30),
            "unsurfaced" => Some(30),
            "living_street" => Some(10),
            "service" => Some(5),
            _ => None,
        },
        None => None,
    }
}
