use geoutils::Location;
use osmpbf::elements::Way;
use osmpbf::elements::WayNodeLocation;
use osmpbf::{Element, ElementReader};
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::time::Instant;

#[derive(Default)]
pub struct RoadNetwork {
    // vertex id is an integer (i64)
    // edge is HashMap of the <NodeId, Cost>
    pub nodes: HashSet<i64>,
    pub edges: HashMap<i64, HashMap<i64, u32>>,
}

pub struct SimplifiedWay {
    id: i64,
    highway_speed_m_per_s: f32,
    node_sequence: Vec<i64>
}

pub fn speed_from_way_kmh(way: &osmpbfreader::objects::Way) -> Option<u32> {

    let tags = way.tags.clone();
    let highway = tags.into_inner().into_iter().find(|(key, _)| key == &"highway");

    match highway {
        Some(highway) => {
            match highway.1.as_str() {
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
            }
        }
        None => None,
    }
}

impl RoadNetwork {
    pub fn read_from_osm_file(path: &str) -> Result<RoadNetwork, Box<dyn Error>> {
        let mut graph = RoadNetwork::new();

        let mut way_counter: u32 = 0;
        let mut node_counter: u32 = 0;
        let mut dense_node_counter: u32 = 0;

        let path_cleaned = std::path::Path::new(&path);
        let r = std::fs::File::open(&path_cleaned).unwrap();

        let mut pbf = osmpbfreader::OsmPbfReader::new(r);

        let mut new_way_counter: u32 = 0;
        let mut new_node_counter: u32 = 0;
        
        use osmpbfreader::objects::OsmObj;
        let mut ways:Vec<SimplifiedWay> = vec![];

        let mut nodes_hashmap: HashMap<i64, Location> = HashMap::new();

        for obj in pbf.iter().map(Result::unwrap) {
            match obj {
                OsmObj::Node(node) => {
                    new_node_counter = new_node_counter + 1;
                    graph.nodes.insert(node.id.0);
                    nodes_hashmap.insert(node.id.0, Location::new(node.lat(), node.lon()));
                },
                OsmObj::Way(way) => {
                    new_way_counter = new_way_counter + 1;

                    if let Some(speed) = speed_from_way_kmh(&way) {
                        let speed_metres_per_second:f32 = speed as f32 * (5.0 / 18.0);
                       // println!("node ref like: {:?}", way.raw_refs());
                    
                        if way.nodes.len() >= 2 {
                            ways.push(SimplifiedWay {
                                node_sequence: Vec::from_iter(way.nodes.into_iter().map(|x| x.0.clone()).collect::<Vec<i64>>()),
                                id: way.id.0,
                                highway_speed_m_per_s: speed_metres_per_second
                            });
                        }
                    }
                },
                _ => {}
            }
        }

        println!("{} new nodes, {} new ways", new_way_counter, new_node_counter); println!("{} simplified way count", ways.len());
        println!("{} in nodes_hashmap",  nodes_hashmap.len());

        for way in ways {
                let mut previous_head_node_location_now_tail_location: Option<&Location> = None;
                let mut previous_head_node_index:usize = 0;
    
                for i in 0..way.node_sequence.len() - 1 {
                    let tail_location:Option<&Location> = match previous_head_node_location_now_tail_location {
                            Some(previous_head_node_location_now_tail_location) => {
                                match previous_head_node_index == i {
                                    true => Some(previous_head_node_location_now_tail_location),
                                    false => nodes_hashmap.get(&way.node_sequence[i])
                                }
                            },
                            None => nodes_hashmap.get(&way.node_sequence[i])
                        };
    
                    if let Some(tail_location) = tail_location {
                        //tail location is found

                        let head_location = nodes_hashmap.get(&way.node_sequence[i + 1]);

                        if let Some(head_location) = head_location {

                            let distance_metres = tail_location
                            .haversine_distance_to(head_location)
                            .meters();

                            let speed_metres_per_second:f32 = way.highway_speed_m_per_s as f32 * (5.0 / 18.0);
                            let cost = (distance_metres / speed_metres_per_second as f64) as u32;
                            
                            let tail_id = way.node_sequence[i];
                            let head_id = way.node_sequence[i + 1];

                            graph.edges.entry(tail_id).and_modify(|edge_list| {edge_list.insert(head_id, cost);}).or_insert({
                                let mut a = HashMap::new();
                                a.insert(head_id, cost);
                                a
                            });
    
                            graph.edges.entry(head_id).and_modify(|edge_list| {edge_list.insert(tail_id, cost);}).or_insert({
                                let mut a = HashMap::new();
                                a.insert(tail_id, cost);
                                a
                            });

                            //save back to prevent relookup
                            previous_head_node_location_now_tail_location = Some(&head_location);
                            previous_head_node_index = i + 1;
                        }
                    }
                }
        }

        Ok(graph)
    }

    pub fn new() -> RoadNetwork {
        RoadNetwork {
            nodes: HashSet::new(),
            edges: HashMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /* 
    #[test]
    fn test_baden_wuerttemberg() {
        test_osm("./baden-wuerttemberg-latest.osm.pbf");
    }*/

    #[test]
    fn ucirvine() {
        test_osm("./uci.osm.pbf");
    }

    #[test]
    fn bast_baden_wuerttemberg() {
        test_osm("./bast-baden-wuerttemberg.pbf");
    }

    fn test_osm(path: &str) -> () {
        let start = Instant::now();
        let baden_graph = RoadNetwork::read_from_osm_file(path);
        let elapsed = start.elapsed();
        println!("{} Elapsed: {:.2?}",path, elapsed);
        assert!(baden_graph.is_ok());

        let baden_graph = baden_graph.unwrap();

        println!(
            "{} {} nodes, {} edges",
            path,
            baden_graph.nodes.len(),
            baden_graph.edges.len()
        );
    }
    
}
