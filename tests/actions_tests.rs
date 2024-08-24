use geo::Coord;
use itertools::Itertools;
use keiro::actions::{ConstVel2D, PointST, routes, Action, Agent, Schedule};
use proptest::prelude::*;


#[test]
fn test_solver() {
    
    let agent1 = Agent {
        name: String::from("agent1"), 
        position: Coord {x: 10.0, y: 10.0 },
        velocity: ConstVel2D {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 0
    };
    let agent2 = Agent {
        name: String::from("agent2"), 
        position: Coord {x: 30.0, y: 10.0 },
        velocity: ConstVel2D {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 1
    };
    let agent3 = Agent {
        name: String::from("agent3"), 
        position: Coord {x: 50.0, y: 10.0 },
        velocity: ConstVel2D {x: 2.0, y: 1.0}, 
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

    // run
    let actual = routes(&agents, schedule);

    // assert
    let agent_paths = actual.routes.iter()
        .sorted_by_key(|(a, _)| a.order)
        .map(|(n, paths)| (n, paths.iter().flat_map(|p| p.to_points_st()).collect::<Vec<PointST>>()))
        .collect::<Vec<(&Agent, Vec<PointST>)>>();

    // serialize for plots
    let ser_paths = agent_paths.clone();
    let str = serde_yaml::to_string(&ser_paths).unwrap();
    let _ = std::fs::write("/home/hendrik/log/paths.yml", str);

    // safety distances
    for (t1, t2) in agent_paths.iter().tuple_windows() {
            let (a1, p1) = t1;
            let (a2, p2) = t2;
            let sd = f64::max(a1.safety_x, a2.safety_x);

            all_first_points_outside_sd(a1, p1, a2, p2, sd);
            all_first_points_outside_sd(a2, p2, a1, p1, sd);
    }

}

fn arb_action(agents: Vec<Agent>) -> impl Strategy<Value = Action> {
    (0..agents.len(), 0..90, 0..50, 1..20).prop_map(move |(i, x, y, d)| {
        let sd = 10.0;
        Action {
            agent: agents[i].clone(),
            target: Coord {
                x: (sd*i as f64 + f64::from(x)), 
                y: f64::from(y)
            },
            duration: f64::from(d)
        }
    })
}

fn arb_schedule() -> impl Strategy<Value = (Vec<Agent>, Schedule)> {
    let agent1 = Agent {
        name: String::from("agent1"), 
        position: Coord {x: 10.0, y: 10.0 },
        velocity: ConstVel2D {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 0
    };
    let agent2 = Agent {
        name: String::from("agent2"), 
        position: Coord {x: 30.0, y: 10.0 },
        velocity: ConstVel2D {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 1
    };
    let agent3 = Agent {
        name: String::from("agent3"), 
        position: Coord {x: 50.0, y: 10.0 },
        velocity: ConstVel2D {x: 2.0, y: 1.0}, 
        safety_x: 10.0, 
        order: 2
    };
    let agents = vec![agent1, agent2, agent3];
    let action_st = arb_action(agents.clone());
    (Just(agents), proptest::collection::vec(action_st, 100).prop_map(|v| Schedule{ actions: v}))

}

proptest! {
    #[test]
    fn test_safety_distances((agents, schedule) in arb_schedule()) {
        // run
        let actual = routes(&agents, schedule);

        // assert
        let agent_paths = actual.routes.iter()
            .sorted_by_key(|(a, _)| a.order)
            .map(|(n, paths)| (n, paths.iter().flat_map(|p| p.to_points_st()).collect::<Vec<PointST>>()))
            .collect::<Vec<(&Agent, Vec<PointST>)>>();

        // serialize for plots
        let ser_paths = agent_paths.clone();
        let str = serde_yaml::to_string(&ser_paths).unwrap();
        let _ = std::fs::write("/home/hendrik/log/paths.yml", str);

        // safety distances
        for (t1, t2) in agent_paths.iter().tuple_windows() {
                let (a1, p1) = t1;
                let (a2, p2) = t2;
                let sd = f64::max(a1.safety_x, a2.safety_x);

                all_first_points_outside_sd(a1, p1, a2, p2, sd);
                all_first_points_outside_sd(a2, p2, a1, p1, sd);
        }
    }
}

fn all_first_points_outside_sd(
    a1: &&Agent, p1: &Vec<PointST>, a2: &&Agent, p2: &Vec<PointST>, sd: f64) -> () {
    for p in p1.iter() {
        let c = interpolate(p, *a2, p2);
        if c.is_some() {
            let cond = if a1.order < a2.order {
                c.unwrap().x - p.x >= sd
            } else {
                p.x - c.unwrap().x >= sd
            };
            assert!(cond);
        }
    }
}

fn interpolate(p: &PointST, a: &Agent, pts: &[PointST]) -> Option<Coord> {
    let oi = pts.iter().position(|pt| p.t < pt.t);
    oi.map(|i| (pts[i-1].clone(), pts[i].clone()))
        .map(|(p1, p2)| {
            let sgn_vx = if p2.x > p1.x { 1.0 } else if p2.x < p1.x { -1.0 } else { 0.0 };
            let sgn_vy = if p2.y > p1.y { 1.0 } else if p2.y < p1.y { -1.0 } else { 0.0 };
            let max_dx = f64::abs(p1.x - p2.x);
            let max_dy = f64::abs(p1.y - p2.y);
            Coord { 
                x: p1.x + sgn_vx * f64::min(a.velocity.x * (p.t - p1.t), max_dx),
                y: p1.y + sgn_vy * f64::min(a.velocity.y * (p.t - p1.t), max_dy)
            }
        })
}


