use core::num;

use geo::Coord;
use itertools::Itertools;
use keiro::actions::{routes, Action, ActionType, Agent, ConstVel2D, PointST, Schedule};
use proptest::prelude::*;

fn arb_action(
    agents: Vec<Agent>,
    sds_l: Vec<f64>,
    sds_r: Vec<f64>,
) -> impl Strategy<Value = Action> {
    (0..agents.len(), 0.0..1.0, 0.0..50.0, 1..20).prop_map(move |(i, x, y, d)| Action {
        agent: agents[i].clone(),
        target: Coord {
            x: sds_l[i] + (sds_r[i] - sds_l[i]) * x,
            y: y,
        },
        duration: f64::from(d),
        r#type: ActionType::Scheduled,
    })
}

fn arb_schedule() -> impl Strategy<Value = (Vec<Agent>, Schedule)> {
    let x_min = 0.0;
    let x_max = 200.0;
    let num_agents = 3;
    let safe_dists = proptest::collection::vec(10.0..20.0, num_agents);
    let agents_st = safe_dists
        .prop_map(move |v| {
            let mut sds_acc_l = vec![x_min];
            for i in 0..(v.len() - 1) {
                sds_acc_l.push(sds_acc_l.last().unwrap() + f64::max(v[i], v[i + 1]));
            }

            let mut v_rev = v.clone();
            v_rev.reverse();
            let mut sds_acc_r = vec![x_max];
            for i in 0..(v_rev.len() - 1) {
                sds_acc_r.push(sds_acc_r.last().unwrap() - f64::max(v[i], v[i + 1]));
            }
            sds_acc_r.reverse();

            let mut agents = vec![];
            for i in 0..v.len() {
                agents.push(Agent {
                    name: format!("agent-{}", i),
                    position: Coord {
                        x: sds_acc_l[i],
                        y: 10.0,
                    },
                    velocity: ConstVel2D { x: 2.0, y: 1.0 },
                    safety_x: v[i],
                    order: i as i64,
                });
            }
            (agents, sds_acc_l, sds_acc_r)
        })
        .boxed();

    let action_st = agents_st
        .clone()
        .prop_flat_map(|(ags, sds_l, sds_r)| {
            (
                Just(ags.clone()),
                proptest::collection::vec(arb_action(ags, sds_l, sds_r), 100)
                    .prop_map(|v| Schedule { actions: v }),
            )
        })
        .boxed();
    action_st
}

proptest! {
    #[test]
    fn test_safety_distances((agents, schedule) in arb_schedule()) {
        // run
        let actual = routes(&agents, schedule);

        // assert
        let agent_paths = actual.routes.iter()
            .sorted_by_key(|(a, _)| a.order)
            .map(|(n, paths)| (n, paths.iter().map(|p| (p.to_points_st(), p.action.r#type.clone())).collect::<Vec<(Vec<PointST>, ActionType)>>()))
            .collect::<Vec<(&Agent, Vec<(Vec<PointST>, ActionType)>)>>();

        // serialize for plots
        let ser_paths = agent_paths.clone();
        let str = serde_yaml::to_string(&ser_paths).unwrap();
        let _ = std::fs::write("/home/hendrik/log/paths.yml", str);

        // safety distances
        for (t1, t2) in agent_paths.iter().tuple_windows() {
                let (a1, p1) = t1;
                let (a2, p2) = t2;
                let sd = f64::max(a1.safety_x, a2.safety_x);
                let pts1 = p1.iter().flat_map(|p| p.0.clone()).collect::<Vec<PointST>>();
                let pts2 = p2.iter().flat_map(|p| p.0.clone()).collect::<Vec<PointST>>();

                all_first_points_outside_sd(a1, &pts1, a2, &pts2, sd);
                all_first_points_outside_sd(a2, &pts2, a1, &pts1, sd);
        }
    }
}

fn all_first_points_outside_sd(
    a1: &&Agent,
    p1: &Vec<PointST>,
    a2: &&Agent,
    p2: &Vec<PointST>,
    sd: f64,
) -> () {
    for p in p1.iter() {
        let c = interpolate(p, *a2, p2);
        if c.is_some() {
            let cond = if a1.order < a2.order {
                c.unwrap().x - p.x >= sd
            } else {
                p.x - c.unwrap().x >= sd
            };
            if !cond {
                println!("{:?}", p)
            }
            assert!(cond);
        }
    }
}

fn interpolate(p: &PointST, a: &Agent, pts: &[PointST]) -> Option<Coord> {
    let oi = pts.iter().position(|pt| p.t < pt.t);
    oi.map(|i| (pts[i - 1].clone(), pts[i].clone()))
        .map(|(p1, p2)| {
            let sgn_vx = if p2.x > p1.x {
                1.0
            } else if p2.x < p1.x {
                -1.0
            } else {
                0.0
            };
            let sgn_vy = if p2.y > p1.y {
                1.0
            } else if p2.y < p1.y {
                -1.0
            } else {
                0.0
            };
            let max_dx = f64::abs(p1.x - p2.x);
            let max_dy = f64::abs(p1.y - p2.y);
            Coord {
                x: p1.x + sgn_vx * f64::min(a.velocity.x * (p.t - p1.t), max_dx),
                y: p1.y + sgn_vy * f64::min(a.velocity.y * (p.t - p1.t), max_dy),
            }
        })
}
