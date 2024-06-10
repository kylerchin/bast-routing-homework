use geoutils::Location;
use osmpbf::elements::Way;
use osmpbf::elements::WayNodeLocation;
use osmpbf::{Element, ElementReader};
use priority_queue::DoublePriorityQueue;
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::sync::Arc;
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
    node_sequence: Vec<i64>,
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

struct DijkstrasAlgorithm {
    graph: RoadNetwork,
    //the value is the round number
    visited_node_marks: HashMap<i64, usize>,
    number_of_completed_rounds: usize,
}

struct ShortestPath {
    path: Vec<i64>,
    cost: u32,
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum BastPriorityValue {
    Infinity,
    Some(u32),
}

impl std::ops::Add for BastPriorityValue {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        match self {
            BastPriorityValue::Infinity => BastPriorityValue::Infinity,
            BastPriorityValue::Some(a) => match rhs {
                BastPriorityValue::Infinity => BastPriorityValue::Infinity,
                BastPriorityValue::Some(b) => BastPriorityValue::Some(a + b),
            },
        }
    }
}

impl PartialOrd for BastPriorityValue {
    fn partial_cmp(&self, other: &BastPriorityValue) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for BastPriorityValue {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match (self, other) {
            (BastPriorityValue::Infinity, BastPriorityValue::Infinity) => std::cmp::Ordering::Equal,
            (BastPriorityValue::Infinity, BastPriorityValue::Some(_)) => {
                std::cmp::Ordering::Greater
            }
            (BastPriorityValue::Some(_), BastPriorityValue::Infinity) => std::cmp::Ordering::Less,
            (BastPriorityValue::Some(a), BastPriorityValue::Some(b)) => a.cmp(b),
        }
    }

    fn max(self, other: Self) -> Self {
        match (self, other) {
            (BastPriorityValue::Infinity, _) => BastPriorityValue::Infinity,
            (_, BastPriorityValue::Infinity) => BastPriorityValue::Infinity,
            (BastPriorityValue::Some(a), BastPriorityValue::Some(b)) => {
                if a > b {
                    BastPriorityValue::Some(a)
                } else {
                    BastPriorityValue::Some(b)
                }
            }
        }
    }

    fn min(self, other: Self) -> Self {
        match (self, other) {
            (BastPriorityValue::Infinity, _) => other,
            (_, BastPriorityValue::Infinity) => self,
            (BastPriorityValue::Some(a), BastPriorityValue::Some(b)) => {
                if a < b {
                    BastPriorityValue::Some(a)
                } else {
                    BastPriorityValue::Some(b)
                }
            }
        }
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        self.max(min).min(max)
    }
}

impl DijkstrasAlgorithm {
    //Compute the shortest path from source to target, returns the cost
    // run until all nodes reachable from the source are settled
    // -1 as target id tries to settle all nodes

    pub fn reduce_to_largest_connected_component(&mut self) -> () {
        let largest_connected_component = self.find_largest_connected_component();

        //delete all nodes and corrosponding edges which are not in the largest connected component

        for (marked_node_id, iteration_component) in self.visited_node_marks.iter() {
            if *iteration_component != largest_connected_component {
                //delete this

                self.graph.edges.remove(marked_node_id);
                self.graph.nodes.remove(marked_node_id);
            }
        }
    }

    pub fn find_largest_connected_component(&mut self) -> usize {
        while self
            .visited_node_marks
            .iter()
            .find(|node_mark| *node_mark.1 == 0)
            .is_some()
        {
            let pick_source_id = self
                .visited_node_marks
                .iter()
                .find(|node_mark| *node_mark.1 == 0)
                .unwrap()
                .0;

            let pick_target_id_pre = self
                .visited_node_marks
                .iter()
                .find(|node_mark| *node_mark.1 == 0 && *node_mark.0 != *pick_source_id);

            if (pick_target_id_pre.is_none()) {
                break;
            }

            let pick_target_id = pick_target_id_pre.unwrap().0;

            println!(
                "Round {}, Now attempting to path {} and {}",
                self.number_of_completed_rounds, pick_source_id, pick_target_id
            );
            self.compute_shortest_path(*pick_source_id, *pick_target_id);
        }

        //scan through visited node marks to make a ranking table

        let mut round_to_number_of_nodes: HashMap<usize, usize> = HashMap::new();

        for (mark_node_id, mark_round_number) in self.visited_node_marks.iter() {
            round_to_number_of_nodes
                .entry(*mark_round_number)
                .and_modify(|x| *x += 1)
                .or_insert(1);
        }

        let mut sorted_round_order: Vec<(usize, usize)> =
            round_to_number_of_nodes.into_iter().collect();

        sorted_round_order.sort_by_key(|k| k.1);

        sorted_round_order.reverse();

        println!(
            "Largest connected components with node count: {:?}",
            sorted_round_order
        );

        sorted_round_order[0].0
    }

    pub fn compute_shortest_path(&mut self, source: i64, target: i64) -> BastPriorityValue {
        self.number_of_completed_rounds = self.number_of_completed_rounds + 1;

        
        // used for finding the largest connected component
        self.visited_node_marks
                    .insert(source, self.number_of_completed_rounds);

        //create vertex priority queue Q
        let mut pq: DoublePriorityQueue<i64, BastPriorityValue> = DoublePriorityQueue::new();

        let mut distances: HashMap<i64, BastPriorityValue> = HashMap::new();
        // Predecessor data store
        let mut prev: HashMap<i64, Option<i64>> = HashMap::new();

        //initialisation
        distances.insert(source, BastPriorityValue::Some(0));

        // associated priority equals dist[·]
        pq.push(source.clone(), BastPriorityValue::Some(0));

        for node in self.graph.nodes.iter() {
            if node != &source {
                prev.insert(node.clone(), None); // Predecessor of v
                                                 //save on memory, don't insert nothing, if nothing is found, state that the node is infinite distance
                                                 //distances.insert(node.clone(), BastPriorityValue::Infinity);  // Unknown distance from source to v
            }
        }

        //the main loop
        while !pq.is_empty() {
            // Remove and return best vertex
            //u ← Q.extract_min()
            if let Some(u) = pq.pop_min() {
                // Go through all v neighbours of u
                if let Some(neighbours) = self.graph.edges.get(&u.0) {
                    for v in neighbours {
                        //u.0 is the node id
                        //distances.get(&u.0).unwrap().clone() is cost of node u
                        //v.1 is cost for v pair, v.0 is the node id
                        let u_dist = match distances.get(&u.0) {
                            Some(u_dist) => u_dist.clone(),
                            None => BastPriorityValue::Infinity,
                        };
                        let alt = u_dist + BastPriorityValue::Some(v.1.clone());

                        if let Some(dist_v) = distances.get(&v.0) {
                            //if the new distance is better than the previously stored distance for this node
                            if alt < *dist_v {
                                prev.insert(v.0.clone(), Some(u.0.clone()));

                                distances.insert(v.0.clone(), alt.clone());

                                //Instead of filling the priority queue with all nodes in the initialization phase,
                                // it is also possible to initialize it to contain only source;
                                //then, inside the if alt < dist[v] block,
                                //the decrease_priority() becomes an add_with_priority() operation if the node is not already in the queue
                                pq.push(v.0.clone(), alt.clone());
                            }
                        }
                    }
                }
            }
        }

        for distance_store in distances.iter() {
            match distance_store.1 {
                BastPriorityValue::Some(distance) => {
                        // used for finding the largest connected component
                        self.visited_node_marks
                            .insert(*distance_store.0, self.number_of_completed_rounds);
                }
                _ => {}
            }
        }

        //return the cost of the target node
        match distances.get(&target) {
            Some(target_cost) => *target_cost,
            None => BastPriorityValue::Infinity,
        }
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
        let mut ways: Vec<SimplifiedWay> = vec![];

        let mut nodes_hashmap: HashMap<i64, Location> = HashMap::new();

        for obj in pbf.iter().map(Result::unwrap) {
            match obj {
                OsmObj::Node(node) => {
                    new_node_counter = new_node_counter + 1;
                    //  graph.nodes.insert(node.id.0);
                    nodes_hashmap.insert(node.id.0, Location::new(node.lat(), node.lon()));
                }
                OsmObj::Way(way) => {
                    new_way_counter = new_way_counter + 1;

                    if let Some(speed) = speed_from_way_kmh(&way) {
                        let speed_metres_per_second: f32 = speed as f32 * (5.0 / 18.0);
                        // println!("node ref like: {:?}", way.raw_refs());

                        if way.nodes.len() >= 2 {
                            ways.push(SimplifiedWay {
                                node_sequence: Vec::from_iter(
                                    way.nodes
                                        .into_iter()
                                        .map(|x| x.0.clone())
                                        .collect::<Vec<i64>>(),
                                ),
                                id: way.id.0,
                                highway_speed_m_per_s: speed_metres_per_second,
                            });
                        }
                    }
                }
                _ => {}
            }
        }

        println!(
            "{} new nodes, {} new ways",
            new_node_counter, new_way_counter
        );
        println!("{} simplified way count", ways.len());

        for way in ways {
            let mut previous_head_node_location_now_tail_location: Option<&Location> = None;
            let mut previous_head_node_index: usize = 0;

            for i in 0..way.node_sequence.len() - 1 {
                let tail_location: Option<&Location> =
                    match previous_head_node_location_now_tail_location {
                        Some(previous_head_node_location_now_tail_location) => {
                            match previous_head_node_index == i {
                                true => Some(previous_head_node_location_now_tail_location),
                                false => nodes_hashmap.get(&way.node_sequence[i]),
                            }
                        }
                        None => nodes_hashmap.get(&way.node_sequence[i]),
                    };

                if let Some(tail_location) = tail_location {
                    //tail location is found

                    let head_location = nodes_hashmap.get(&way.node_sequence[i + 1]);

                    if let Some(head_location) = head_location {
                        let distance_metres =
                            tail_location.haversine_distance_to(head_location).meters();

                        let speed_metres_per_second: f32 =
                            way.highway_speed_m_per_s as f32 * (5.0 / 18.0);
                        let cost = (distance_metres / speed_metres_per_second as f64) as u32;

                        let tail_id = way.node_sequence[i];
                        let head_id = way.node_sequence[i + 1];

                        graph
                            .edges
                            .entry(tail_id)
                            .and_modify(|edge_list| {
                                edge_list.insert(head_id, cost);
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(head_id, cost);
                                a
                            });

                        graph
                            .edges
                            .entry(head_id)
                            .and_modify(|edge_list| {
                                edge_list.insert(tail_id, cost);
                            })
                            .or_insert({
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

        //new node insertion process
        let new_nodes: HashSet<i64> =
            HashSet::from_iter(graph.edges.iter().map(|(node_id, _)| node_id.clone()));

        graph.nodes = new_nodes;

        println!("{} in nodes_hashmap", nodes_hashmap.len());

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
        let graph = test_osm("./uci.osm.pbf");

        //for (node_id, edges) in graph.edges {
        //    println!("{} | {:?}", node_id, edges);
        //}

        let initial_visited_node_marks = {
            let mut visited_node_marks: HashMap<i64, usize> = HashMap::new();

            for node in &graph.nodes {
                visited_node_marks.insert(*node, 0);
            }

            visited_node_marks
        };

        assert!(graph.nodes.contains(&1834861939));
        assert!(graph.nodes.contains(&3710901043));

        let mut routing = DijkstrasAlgorithm {
            graph: graph,
            visited_node_marks: initial_visited_node_marks,
            number_of_completed_rounds: 0,
        };


        let route_between_shen_and_ben = routing.compute_shortest_path(1834861939, 3710901043);

        println!("Cost in seconds between Shen and Ben {:?}", route_between_shen_and_ben);

        //find largest connected component
      //  let start_connected_component_compute = Instant::now();
      //  let largest_connected_component = routing.find_largest_connected_component();

       // let end_connected_component_compute_time = Instant::now();

        /*
        println!(
            "Duration to find connected components {:?}",
            end_connected_component_compute_time - start_connected_component_compute
        );
         */
    }

    #[test]
    fn bast_baden_wuerttemberg() {
        test_osm("./bast-baden-wuerttemberg.pbf");
    }

    fn test_osm(path: &str) -> RoadNetwork {
        let start = Instant::now();
        let graph = RoadNetwork::read_from_osm_file(path);
        let elapsed = start.elapsed();
        println!("{} Elapsed: {:.2?}", path, elapsed);
        assert!(graph.is_ok());

        let graph = graph.unwrap();

        println!(
            "{} {} nodes, {} edges",
            path,
            graph.nodes.len(),
            graph.edges.len()
        );

        graph
    }
}
