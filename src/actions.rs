use geo::{Coord, Polygon};
use itertools::Itertools;
use serde::{Deserialize, Serialize};
use std::{cmp::Ordering, collections::HashMap};

/// An agent is a named entity that can execute actions
#[derive(Clone)]
pub struct Agent {
    pub name: String,
    pub position: Coord,
    pub velocity: Coord,
    pub safety_x: f64,
    pub order: i64
}

/// An action is an event that is executed by an agent at a given location.
#[derive(Clone)]
pub struct Action {
    pub agent: Agent,
    pub target: Coord,
    pub duration: f64,
}

/// A schedule is a list of events, determining the absolute order in which
/// they have to be executed.
pub struct Schedule {
    pub actions: Vec<Action>
}

#[derive(Clone)]
pub struct Segment {
    pub start: Coord,
    pub end: Coord,
    pub duration: f64
}

/// A path is a list of moves necessary to arrive at the given action.
#[derive(Clone)]
pub struct Path {
    pub moves: Vec<Segment>,
    pub action: Action,
    pub t_start: f64,
    pub t_end: f64
}

impl Path {
    fn start(&self) -> Coord {
        match self.moves.iter().next() {
            Some(s) => s.start,
            None => self.action.target
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct PointST {
    pub x: f64,
    pub y: f64,
    pub t: f64
}

impl Path {
    pub fn to_points_st(&self) -> Vec<PointST> {
        let mut result = match self.moves.first() {
            Some(s) => vec![PointST {x: s.start.x, y: s.start.y, t: self.t_start} ],
            None => vec![]
        };
        let mut clock = self.t_start;
        for s in self.moves.iter() {
            clock = clock + s.duration;
            result.push(PointST {x: s.end.x, y: s.end.y, t: clock})
        }
        result.push(PointST {
            x: self.action.target.x, 
            y: self.action.target.y, 
            t: clock + self.action.duration
        });
        result
    }
}

enum ConflictResolution {
    LeftOf(f64),
    RightOf(f64)
}

struct Conflict<'a> {
    cause: &'a Action,
    resolution: ConflictResolution
}

pub struct Routing {
    /// The list of paths for each agent.
    pub routes: Vec<(Agent, Vec<Path>)>
}

/// Compute routes for each agent, given a schedule of concrete actions
pub fn run(agents: &Vec<Agent>, sched: Schedule) -> Routing {

    let init: Vec<(Agent, Vec<Path>)> 
        = agents.iter().map(|a| (
            a.clone(), 
            vec![Path {
                moves: vec![], 
                action: Action {agent: a.clone(), target: a.position, duration: 0.0 },
                t_start: 0.0, 
                t_end: 0.0
            }]
        )).collect();
    let r = sched.actions.iter().fold(
        init,
        |acc, a| execute_action(a, acc)
    );

    Routing {
        routes: r
    }
}

fn execute_action(
    action: &Action, r: Vec<(Agent, Vec<Path>)>
) -> Vec<(Agent, Vec<Path>)> {
    let agent_paths = get_paths(&action.agent, &r);
    let path_2d = find_path_2d(
        action, 
        agent_paths.iter().last().unwrap().action.target.clone()
    );

    let mut result = r.clone();
    while let Some(conflict) = first_conflict(&action.agent, &path_2d, &result) {
        let ev_action = evasion_target(&conflict);
        result = execute_action(&ev_action, result.clone());
    }
    let idle = idle_path(action, &path_2d, &result);
    let path = Path {
        moves: path_2d.clone(),
        action: action.clone(),
        t_start: idle.t_end,
        t_end: idle.t_end + path_2d[0].duration + action.duration
    };
    let mut v = get_paths(&action.agent, &result).clone();
    if idle.t_end != idle.t_start {
        v.push(idle);
    }
    v.push(path);
    let i = r.iter().position(|(a, _)| a.name == action.agent.name).unwrap();
    result[i] = (action.agent.clone(), v);
    result
}

fn get_paths<'a, 'b>(agent: &'a Agent, r: &'b Vec<(Agent, Vec<Path>)>) -> &'b Vec<Path> {
    let (_, last_path) = r.iter().find(|(a, _)| a.name == agent.name).unwrap();
    last_path
}

fn idle_path(action: &Action, path_2d: &Vec<Segment>, r: &Vec<(Agent, Vec<Path>)>) -> Path {
    let last_path = get_paths(&action.agent, r).last().unwrap();
    let t0 = last_path.t_end;
    let duration = path_2d[0].duration;
    let xi = path_2d[0].start.x;
    let xf = path_2d[0].end.x;
    let sd = action.agent.safety_x;
    let s = r.iter()
        .filter(|(a, _)| a.name != action.agent.name)
        .filter(|(_, ps)| ps.iter().any(|p| p.t_end >= t0))
        .map(|(a, ps)| (a, ps.iter().skip_while(|p| p.t_end < t0)))
        .map(|(a, ps)| (
            a, ps
                .flat_map(|p| p.to_points_st())
                .tuple_windows()
                .filter(|(p1, _)| if a.order < action.agent.order { 
                        xf - p1.x < sd
                    } else { 
                        p1.x - xf < sd 
                })
                .last()
        ))
        .filter(|(_, t)| t.is_some())
        .map(|(a, t)| (a, t.unwrap()))
        .map(|(a, (p1, _))| {
            let t1 = p1.t - (f64::abs(p1.x - xi) - sd) / action.agent.velocity.x;
            let t2 = p1.t + (sd - f64::abs(p1.x - xf) - sd) / a.velocity.x - duration; 
            t1.max(t2)
        })
        .reduce(f64::max)
        .unwrap_or(t0)
        .ceil();

    Path{
        moves: Vec::new(),
        action: Action {
            agent: action.agent.clone(),
            target: last_path.action.target,
            duration: s - t0,
        },
        t_start: t0,
        t_end: s
    }
}

fn evasion_target(conflict: &Conflict) -> Action {
    let target_x = match conflict.resolution {
        ConflictResolution::LeftOf(l) => l,
        ConflictResolution::RightOf(l) => l
    };
    Action {
        agent: conflict.cause.agent.clone(),
        target: Coord { x: target_x, y: conflict.cause.target.y },
        duration: 0.0,
    }
}

fn first_conflict<'a>(
    agent: &'a Agent, path: &'a Vec<Segment>, r: &'a Vec<(Agent, Vec<Path>)>
) -> Option<Conflict<'a>> {
    let xs = path.iter().map(|s| s.end.x).collect::<Vec<f64>>();
    let min_x = xs.clone().into_iter().reduce(f64::min).unwrap();
    let max_x = xs.into_iter().reduce(f64::max).unwrap();
    let result = r.iter()
        .filter(|(a, _)| a.name != agent.name)
        .map(|(_, paths)| &paths.iter().last().unwrap().action)
        .map(|a| (
            a, 
            if a.agent.order < agent.order && a.target.x > min_x - a.agent.safety_x { 
                Some(ConflictResolution::LeftOf(min_x - a.agent.safety_x))
            } else if a.agent.order > agent.order && a.target.x < max_x + a.agent.safety_x{ 
                Some(ConflictResolution::RightOf(max_x + a.agent.safety_x))
            } else {
                None
            }))
        .filter(|(_, c)| c.is_some())
        .map(|(a, c)| Conflict { cause: a, resolution: c.unwrap() })
        .min_by(|c1, c2| {
            match c1.resolution {
                ConflictResolution::LeftOf(l1) => match c2.resolution {
                    ConflictResolution::LeftOf(l2) => if l1 > l2 { Ordering::Less } else { Ordering:: Greater },
                    ConflictResolution::RightOf(_) => Ordering::Less
                },
                ConflictResolution::RightOf(l1) => match c2.resolution {
                    ConflictResolution::RightOf(l2) => if l1 < l2 { Ordering::Less } else { Ordering:: Greater },
                    ConflictResolution::LeftOf(_) => Ordering::Greater
                }
            }});
        result
}

fn find_path_2d(a: &Action, p: Coord) -> Vec<Segment> {
    let v = a.agent.velocity;
    let t_x = (a.target.x - p.x).abs() / v.x;
    let t_y = (a.target.y - p.y).abs() / v.y;
    let t = t_x.max(t_y);
    vec![Segment {start: p, end: a.target.clone(), duration: t}]
}
