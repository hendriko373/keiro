use geo::{Coord, CoordNum, LineString, Polygon};
use serde::{Deserialize, Serialize};

/// An agent is a named entity that can execute actions
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Agent {
    pub name: String,
    #[serde(with = "PolygonSerde")]
    pub reach: Polygon,
    #[serde(with = "CoordSerde")]
    pub position: Coord<f64>,
    pub velocity: ConstVel2D,
    pub safety_x: f64,
    pub order: i64,
}

/// Motion of constant velocity in two dimensions.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ConstVel2D {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActionType {
    Scheduled,
    Evasive,
    Idle,
}

/// An action is an event that is executed by an agent at a given location.
#[derive(Clone, Debug)]
pub struct Action {
    pub agent: Agent,
    pub target: Coord,
    pub duration: f64,
    pub r#type: ActionType,
}

impl Agent {
    pub fn safety_x(&self, other: &Agent) -> f64 {
        f64::max(self.safety_x, other.safety_x)
    }
}

/// A schedule is a list of events, determining the absolute order in which
/// they have to be executed.
#[derive(Debug)]
pub struct Schedule {
    pub actions: Vec<Action>,
}

#[derive(Clone, Copy)]
pub struct Segment {
    pub start: Coord,
    pub end: Coord,
    pub duration: f64,
}

/// A path is a list of moves necessary to arrive at the given action.
#[derive(Clone)]
pub struct Path {
    /// A list of moves that take the agent from the previous action to the `action`
    pub moves: Vec<Segment>,

    /// The action to be done
    pub action: Action,

    /// The start time of the path, which equals the end time of the previous path
    pub t_start: f64,

    /// The end time of the path, which is after the present action is finished
    pub t_end: f64,
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct PointST {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

#[derive(Serialize, Deserialize, Clone)]
#[serde(remote = "Coord")]
struct CoordSerde<T = f64>
where
    T: CoordNum,
{
    x: T,
    y: T,
}

#[derive(Serialize, Deserialize, Clone)]
struct CoordSerdeForPolygon<T = f64>
where
    T: CoordNum,
{
    x: T,
    y: T,
}

impl<T: CoordNum> From<CoordSerdeForPolygon<T>> for Coord<T> {
    fn from(value: CoordSerdeForPolygon<T>) -> Self {
        Coord {
            x: value.x,
            y: value.y,
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(remote = "Polygon")]
struct PolygonSerde<T = f64>
where
    T: CoordNum,
{
    #[serde(getter = "PolygonSerde::get_exterior")]
    exterior: Vec<CoordSerdeForPolygon<T>>,
    #[serde(getter = "PolygonSerde::get_interiors")]
    interiors: Vec<Vec<CoordSerdeForPolygon<T>>>,
}

impl<T: CoordNum> PolygonSerde<T> {
    fn get_exterior(pol: &Polygon<T>) -> Vec<CoordSerdeForPolygon<T>> {
        pol.exterior()
            .clone()
            .0
            .into_iter()
            .map(|c| CoordSerdeForPolygon { x: c.x, y: c.y })
            .collect()
    }

    fn get_interiors(pol: &Polygon<T>) -> Vec<Vec<CoordSerdeForPolygon<T>>> {
        pol.interiors()
            .into_iter()
            .map(|ls| {
                ls.clone()
                    .0
                    .into_iter()
                    .map(|c| CoordSerdeForPolygon { x: c.x, y: c.y })
                    .collect()
            })
            .collect()
    }
}

impl<T: CoordNum> From<PolygonSerde<T>> for Polygon<T> {
    fn from(value: PolygonSerde<T>) -> Self {
        Polygon::new(
            LineString::from(value.exterior.clone()),
            value
                .interiors
                .into_iter()
                .map(|v| LineString::from(v.clone()))
                .collect(),
        )
    }
}
