use geo::Coord;
use itertools::Itertools;
use keiro::actions::{self, run, Action, Agent, Path, Schedule};


#[test]
fn test_solver() {
    
    let agent1 = Agent {
        name: String::from("agent1"), 
        position: Coord {x: 10.0, y: 10.0 },
        velocity: Coord {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 0
    };
    let agent2 = Agent {
        name: String::from("agent2"), 
        position: Coord {x: 30.0, y: 10.0 },
        velocity: Coord {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 1
    };
    let agent3 = Agent {
        name: String::from("agent3"), 
        position: Coord {x: 50.0, y: 10.0 },
        velocity: Coord {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 2
    };

    let schedule = Schedule {
        actions: vec![
            Action {
                agent: agent1.clone(),
                target: Coord {x: 20.0, y: 20.0 },
                duration: 6.0
            },
            Action {
                agent: agent1.clone(),
                target: Coord {x: 30.0, y: 20.0 },
                duration: 6.0
            },
            Action {
                agent: agent2.clone(),
                target: Coord {x: 20.0, y: 20.0 },
                duration: 6.0
            },
            Action {
                agent: agent3.clone(),
                target: Coord {x: 30.0, y: 20.0 },
                duration: 6.0
            },
            Action {
                agent: agent2.clone(),
                target: Coord {x: 40.0, y: 20.0 },
                duration: 6.0
            },
        ]
    };
    let agents = vec![agent1, agent2, agent3];
    let actual = run(&agents, schedule);
    let agent_paths = actual.routes.iter()
        .map(|(n, paths)| (agents.iter().find(|a| a.name == *n).unwrap(), paths))
        .sorted_by_key(|(a, _)| a.order)
        .collect::<Vec<(&Agent, &Vec<Path>)>>();
    let a = 1;

}

struct PointST {
    x: f64,
    y: f64,
    t: f64
}

fn path_to_spacetime(path: &Path) -> Vec<PointST> {
    let mut result = match path.moves.first() {
        Some(s) => vec![PointST {x: s.start.x, y: s.start.y, t: path.t_start} ],
        None => vec![]
    };
    let mut clock = path.t_start;
    for s in path.moves.iter() {
        clock = clock + s.duration;
        result.push(PointST {x: s.end.x, y: s.end.y, t: clock})
    }
    result.push(PointST {
        x: path.action.target.x, 
        y: path.action.target.y, 
        t: clock + path.action.duration
    });
    result
}