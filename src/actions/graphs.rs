use geo::{Contains, Coord};
use petgraph::{graph::UnGraph, Graph};

use crate::actions::data::Action;


pub fn create_graph(a: &Action, start: Coord) -> UnGraph<Coord<f64>, f64> {
    let internal_g = nodes_edges(a, start);
    create_pet_graph(&internal_g)
}


/// Internal unidirected graph
pub struct G {
    /// The nodes in the graph. The usize in the tuple holds a mapping
    /// from the coordinates to an abstract index.
    nodes: Vec<(Coord<f64>, usize)>,
    /// The edges between the nodes, using the abstract index in `nodes`.
    edges: Vec<(usize, f64)>
}


/// Given an action and a start point, find all undirected edges between the points
/// in the action's reach polygon, retaining only those edges that are completely
/// within the reach polygon. Returns a HashMap where each key is a point with
/// outgoing edges, and the value is a tuple of (neighbor, weight).
fn nodes_edges(a: &Action, start: Coord) -> G {
    
    let poly = &a.agent.reach;
    let mut coords = poly.exterior().points().map(|p| p.0).collect::<Vec<_>>();
    for interior in poly.interiors() {
        coords.extend(interior.points().map(|p| p.0));
    }
    coords.push(start);
    coords.push(a.target);

    let mut nodes = Vec::<(Coord<f64>, usize)>::new();
    let mut edges = Vec::<(usize, f64)>::new();

    for (i, &p1) in coords.iter().enumerate() {
        for (j, &p2) in coords.iter().enumerate() {
            // undirected graph
            if i < j {
                let line_string = geo::LineString::from(vec![p1, p2]);
                if poly.contains(&line_string) {
                    nodes.push((p1, i));
                    edges.push((j, 1.0));
                }
            }
        }
    }
    G { nodes: nodes, edges: edges }
}

fn create_pet_graph(g: &G) -> UnGraph<Coord<f64>, f64> {
    let mut pet_g = Graph::new_undirected();
    let mut pet_ns = Vec::new();
    for (n, _) in g.nodes.iter() {
        let pet_n = pet_g.add_node(*n);
        pet_ns.push(pet_n);
    }
    for (i, e) in g.edges.iter().enumerate() {
        pet_g.add_edge(pet_ns[i], pet_ns[e.0], e.1);
    }
    pet_g
}