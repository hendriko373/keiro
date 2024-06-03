use std::{cmp::Ordering, collections::HashMap};
use geo::{Coord, Polygon};

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
    pub routes: HashMap<String, Vec<Path>>
}

/// Compute routes for each agent, given a schedule of concrete actions
pub fn run(agents: &Vec<Agent>, sched: Schedule) -> Routing {

    let init: HashMap<String, Vec<Path>> 
        = agents.iter().map(|a| (
            a.name.clone(), 
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
    action: &Action, r: HashMap<String, Vec<Path>>
) -> HashMap<String, Vec<Path>> {
    let path_2d = find_path_2d(
        action, 
        r[&action.agent.name].iter().last().unwrap().action.target.clone()
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
    let mut v = result[&action.agent.name].clone();
    if idle.t_end != idle.t_start {
        v.push(idle);
    }
    v.push(path);
    result.insert(action.agent.name.clone(), v);
    result
}

fn idle_path(action: &Action, path_2d: &Vec<Segment>, r: &HashMap<String, Vec<Path>>) -> Path {
    let last_path = r[&action.agent.name].last().unwrap();
    let t0 = last_path.t_end;
    let duration = path_2d[0].duration;
    let xi = path_2d[0].start.x;
    let xf = path_2d[0].end.x;
    let sd = action.agent.safety_x;
    let s = r.iter()
        .filter(|(n, _)| n != &&action.agent.name)
        .filter(|(_, ps)| ps.iter().any(|p| p.t_end >= t0))
        .map(|(_, ps)| ps.iter().skip_while(|p| p.t_end < t0))
        .map(|ps| ps
            .flat_map(|p| {
                let mut acc = vec![(p.t_start, p.start())];
                for m in p.moves.iter() {
                    acc.push((acc.last().unwrap().0 + m.duration, m.end));
                }
                acc
            })
            .collect::<Vec<(f64, Coord)>>()
            .windows(2)
            .map(|l| (l[0], l[1]))
            .filter(|(p1, _)| f64::abs(p1.1.x - xf) < sd)
            .last()
        )
        .filter(|t| t.is_some())
        .map(|t| t.unwrap())
        .map(|(p1, _)| {
            let t1 = p1.0 - (f64::abs(p1.1.x - xi) - sd) / action.agent.velocity.x;
            let t2 = p1.0 + (sd - f64::abs(p1.1.x - xf) - sd) / action.agent.velocity.x - duration; 
                // should be velocity of other agent
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
    agent: &'a Agent, path: &'a Vec<Segment>, r: &'a HashMap<String, Vec<Path>>
) -> Option<Conflict<'a>> {
    let float_cmp = |f1: &&f64, f2: &&f64| f1.partial_cmp(f2).unwrap();
    let xs = path.iter().map(|s| s.end.x).collect::<Vec<f64>>();
    let min_x = *xs.iter().min_by(float_cmp).unwrap();
    let max_x = *xs.iter().max_by(float_cmp).unwrap();
    let result = r.iter()
        .filter(|(n, _)| n != &&agent.name)
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
