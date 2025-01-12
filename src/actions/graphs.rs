use geo::{Contains, Coord, Line, Polygon};
use itertools::Itertools;

pub fn create_graph(_vs: &Vec<Coord>, _es: &Vec<(Coord, Coord, f64)>) {}

pub fn all_edges(vertices: &Vec<Coord>, reach: &Polygon) -> Vec<(geo::Coord, geo::Coord)> {
    let pts = vertices
        .iter()
        .combinations(2)
        .filter(|v| v[0] != v[1])
        .map(|v| Line::new(v[0].clone(), v[1].clone()))
        .filter(|l| reach.contains(l))
        .map(|l| (l.start, l.end))
        .collect();
    pts
}

pub fn get_reach_vertices(reach: &Polygon) -> Vec<Coord> {
    reach
        .exterior()
        .points()
        .chain(reach.interiors().iter().flat_map(|l| l.points()))
        .map(|p| p.into())
        .collect()
}
