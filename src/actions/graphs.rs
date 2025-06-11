use geo::{Contains, Coord, LineString};
use petgraph::{algo::{astar}, graph::{NodeIndex, UnGraph}, Graph};

use crate::actions::data::Action;


pub fn find_path_2d_g(a: &Action, start: Coord) -> () {
    let internal_g = internal_graph(a, start);
    let (pet_g, node_ixs)  = pet_graph(&internal_g);
    let path =  astar(
        &pet_g, node_ixs[0], |n| n == node_ixs[1], |e| *e.weight(), |_| 0);
    match path {
        Some((time, path)) => { () },
        None => ()
    }
}


/// Internal undirected graph
pub struct G {
    /// The nodes. 
    pub nodes: Vec<Coord<f64>>,
    /// The edges. The i-th element contains tuples `(j, wij)`, which connect
    /// `nodes[i]`and `nodes[j]` with weight `wij`.
    pub edges: Vec<Vec<(usize, f64)>>
}


fn internal_graph(a: &Action, start: Coord) -> G {
    
    let poly = &a.agent.reach;
    let mut coords = vec![start, a.target];
    coords.extend(poly.exterior().points().map(|p| p.0));
    for interior in poly.interiors() {
        coords.extend(interior.points().map(|p| p.0));
    }

    let edges = coords.iter()
        .map(|pi| coords.iter().enumerate()
            .filter(|(_, &pj)| poly.contains(&LineString::from(vec![*pi, pj])))
            .map(|(j, _)| (j, 1.0))
            .collect::<Vec<_>>())
        .collect::<Vec<_>>();

    G { nodes: coords, edges: edges }
}

fn pet_graph(g: &G) -> (UnGraph<Coord<f64>, f64>, Vec<NodeIndex>) {
    let mut pet_g = Graph::new_undirected();
    let mut pet_ns = Vec::new();
    for n in g.nodes.iter() {
        let pet_n = pet_g.add_node(*n);
        pet_ns.push(pet_n);
    }
    for (i, es) in g.edges.iter().enumerate() {
        for e in es.iter() {
            pet_g.add_edge(pet_ns[i], pet_ns[e.0], e.1);
        }
    }
    (pet_g, pet_ns)
}