use std::collections::HashMap;

use geo::{Contains, Coord, LineString};
use itertools::Itertools;
use petgraph::{
    algo::astar,
    graph::{NodeIndex, UnGraph},
    Graph,
};

use crate::actions::data::{Action, ConstVel2D, Segment};

pub fn find_path_2d_g(a: &Action, start: Coord) -> Option<Vec<Segment>> {
    let g = create_graph(a, start);
    let path = astar(
        &g.pet_g,
        g.node_to_nix[0],
        |n| n == g.node_to_nix[1],
        |e| *e.weight(),
        |_| 0.0,
    );
    path.map(|(_, path)| {
        path.into_iter()
            .tuple_windows()
            .map(|(s, e)| (s, e, g.pet_g.find_edge(s, e).unwrap()))
            .map(|(s, e, edge_ix)| (s, e, *g.pet_g.edge_weight(edge_ix).unwrap()))
            .map(|(s, e, time)| Segment {
                start: g.nodes[g.nix_to_node[&s]],
                end: g.nodes[g.nix_to_node[&e]],
                duration: time,
            })
            .collect::<Vec<_>>()
    })
}

/// Internal undirected graph
pub struct G {
    /// The nodes. The first node should be the start point, the second the target.
    pub nodes: Vec<Coord<f64>>,
    pub pet_g: UnGraph<Coord<f64>, f64>,
    pub node_to_nix: Vec<NodeIndex>,
    /// A mapping from the pet graph node indices to the element in `nodes`.
    pub nix_to_node: HashMap<NodeIndex, usize>,
}

fn create_graph(a: &Action, start: Coord) -> G {
    let poly = &a.agent.reach;
    let mut coords = vec![start, a.target];
    coords.extend(poly.exterior().points().map(|p| p.0));
    for interior in poly.interiors() {
        coords.extend(interior.points().map(|p| p.0));
    }

    let edges = coords
        .iter()
        .map(|pi| {
            coords
                .iter()
                .enumerate()
                .filter(|(_, &pj)| poly.contains(&LineString::from(vec![*pi, pj])))
                .map(|(j, pj)| (j, timer(*pi, *pj, &a.agent.velocity)))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut pet_g = Graph::new_undirected();
    let mut pet_ns = Vec::new();
    let mut nix_to_coord = HashMap::new();
    for (i, n) in coords.iter().enumerate() {
        let pet_n = pet_g.add_node(*n);
        pet_ns.push(pet_n);
        nix_to_coord.insert(pet_n, i);
    }
    for (i, es) in edges.iter().enumerate() {
        for e in es.iter() {
            pet_g.add_edge(pet_ns[i], pet_ns[e.0], e.1);
        }
    }

    G {
        nodes: coords,
        pet_g: pet_g,
        node_to_nix: pet_ns,
        nix_to_node: nix_to_coord,
    }
}

fn timer(start: Coord<f64>, end: Coord<f64>, vel: &ConstVel2D) -> f64 {
    let t_x = (end.x - start.x).abs() / vel.x;
    let t_y = (end.y - start.y).abs() / vel.y;
    t_x.max(t_y)
}

#[cfg(test)]
mod tests;
