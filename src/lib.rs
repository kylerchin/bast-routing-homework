use std::error::Error;
use std::collections::{HashSet,HashMap};

#[derive(Default,New)]
pub struct RoadNetwork {
 // vertex id is an integer (i32)
 // edge is tuple (i32, i32)
 pub nodes: HashSet<i32>,
 pub edges: HashMap<i32, HashSet<i32>>
}

impl RoadNetwork {
    pub fn read_from_osm_file(path: &str) -> Result<RoadNetwork, Box<dyn Error>> {
        let mut new = RoadNetwork::new();
    }

    pub fn new() -> RoadNetwork {
        RoadNetwork {
            nodes: HashSet::new(),
            edges: HashMap::new()
        }
    }
}

#[cfg(tests)]
mod tests {
    #[test]
    fn test_baden_wuerttemberg() {
        let baden_graph = RoadNetwork::read_from_osm_file("./baden-wuerttemberg.osm");

        assert!(baden_graph.is_ok());
    }
}