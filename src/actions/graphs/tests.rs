use geo::{Coord, LineString, Polygon};

use crate::actions::{
    data::{Action, ActionType, Agent, ConstVel2D, Segment},
    graphs::{find_path_2d_g, timer},
};

#[test]
fn test_find_path_2d_g() {
    let start = Coord { x: 10.0, y: 10.0 };
    let agent = Agent {
        name: String::from("agent"),
        reach: Polygon::new(
            LineString::from(vec![(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]),
            vec![],
        ),
        position: start,
        velocity: ConstVel2D { x: 2.0, y: 1.0 },
        safety_x: 10.0,
        order: 0,
    };
    let target = Coord { x: 90.0, y: 90.0 };
    let action = Action {
        agent: agent.clone(),
        target: target,
        duration: 10.0,
        r#type: ActionType::Scheduled,
    };

    let expected = Some(vec![Segment {
        start: start,
        end: target,
        duration: timer(start, target, &agent.velocity),
    }]);

    let actual = find_path_2d_g(&action, start);

    assert_eq!(actual, expected);
}
