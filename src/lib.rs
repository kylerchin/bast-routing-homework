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

pub fn way_node_to_geoutil_loc(x: &WayNodeLocation) -> Location {
    Location::new(x.lat(), x.lon())
}

pub fn speed_from_way_kmh(way: &Way) -> Option<u32> {
    let highway = way.tags().find(|(key, _)| key == &"highway");

    match highway {
        Some(highway) => match highway.1 {
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

impl RoadNetwork {
    pub fn read_from_osm_file(path: &str) -> Result<RoadNetwork, Box<dyn Error>> {
        let mut graph = RoadNetwork::new();

        let mut way_counter: u32 = 0;
        let mut node_counter: u32 = 0;
        let mut dense_node_counter: u32 = 0;

        let reader_first = ElementReader::from_path(path)?;

        let mut ways:Vec<SimplifiedWay> = vec![];

        let mut nodes_hashmap: HashMap<i64, Location> = HashMap::new();

        reader_first.for_each(|element| match element {
            Element::Node(node) => {
                node_counter += 1;
            },
            Element::DenseNode(dense_node) => {
                dense_node_counter += 1;
                let highway = dense_node.tags().find(|(key, _)| key == &"highway");

                if highway.is_some() {
                    nodes_hashmap.insert(dense_node.id, Location::new(dense_node.lat(), dense_node.lon()));
                }

            },
            Element::Way(way) => {
                way_counter = way_counter + 1;

                if let Some(speed) = speed_from_way_kmh(&way) {
                    let speed_metres_per_second:f32 = speed as f32 * (5.0 / 18.0);

                    for node_id in way.raw_refs() {
                        graph.nodes.insert(node_id.clone());
                    }
                
                    if way.raw_refs().len() >= 2 {
                        ways.push(SimplifiedWay {
                            node_sequence: Vec::from_iter(way.raw_refs().into_iter().map(|x| x.clone()).collect::<Vec<i64>>()),
                            id: way.id(),
                            highway_speed_m_per_s: speed_metres_per_second
                        });
                    }
                }
            },
            _=>{}
        })?;

        println!("{} nodes, {} dense nodes, {} ways", node_counter, dense_node_counter, way_counter);
        println!("{} simplified way count", ways.len());

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


                            let cost = (speed_metres_per_second as f64 / distance_metres) as u32;
                            
                            let tail_id = way.node_sequence[i];
                            let head_id =  way.node_sequence[i + 1];

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
                            previous_head_node_index = i;
                        }
                    }
                }
        }

        /* 
        reader.for_each(|element| match element {
            Element::Way(way) => {

                if let Some(speed) = speed_from_way_kmh(&way) {
                    let speed_metres_per_second:f32 = speed as f32 * (5.0 / 18.0);

                for node_id in way.raw_refs() {
                    graph.nodes.insert(node_id.clone());
                }

                if way.raw_refs().len() >= 2 {
                    let path = way.node_locations().collect::<Vec<WayNodeLocation>>();

                    if path.len() >= 2 {
                    for i in 0..path.len() - 1 {
                        let tail_node_coord = way_node_to_geoutil_loc(&path[i]);
                        let head_node_coord = way_node_to_geoutil_loc(&path[i + 1]);
                        let distance_metres = tail_node_coord
                            .haversine_distance_to(&head_node_coord)
                            .meters();

                        let cost = (speed_metres_per_second as f64 / distance_metres) as u32;

                        let tail_id = way.raw_refs()[i];
                        let head_id = way.raw_refs()[i + 1];

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
                    }
                    } else {
                        //println!("Path string is too short {:?}", path.len());
                    }
                } else {
                       // println!("Way is too short {:?}", way.raw_refs());
                }
                }
            }
            _ => {}
        })?;*/

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

    
    #[test]
    fn test_baden_wuerttemberg() {
        let start = Instant::now();
        let baden_graph = RoadNetwork::read_from_osm_file("./baden-wuerttemberg-latest.osm.pbf");
        let elapsed = start.elapsed();
        println!("Elapsed: {:.2?}", elapsed);
        assert!(baden_graph.is_ok());

        let baden_graph = baden_graph.unwrap();

        println!(
            "baden {} nodes, {} edges",
            baden_graph.nodes.len(),
            baden_graph.edges.len()
        );
    }

    /* 
    #[test]
    fn test_socal() {
        let start = Instant::now();
        let baden_graph = RoadNetwork::read_from_osm_file("./socal-latest.osm.pbf");
        let elapsed = start.elapsed();
        println!("Elapsed: {:.2?}", elapsed);
        assert!(baden_graph.is_ok());

        let baden_graph = baden_graph.unwrap();

        println!(
            "socal {} nodes, {} edges",
            baden_graph.nodes.len(),
            baden_graph.edges.len()
        );
    }
    */
}
