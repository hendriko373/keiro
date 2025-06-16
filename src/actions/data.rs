use geo::{Coord, CoordNum, LineString, Polygon};
use serde::{Deserialize, Serialize};

/// An agent is a named entity that can execute actions
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Agent {
    /// The name of the agent
    pub name: String,
    /// The reach of the agent, represented as a polygon
    #[serde(with = "PolygonSerde")]
    pub reach: Polygon,
    /// The position of the agent, represented as a coordinate
    #[serde(with = "CoordSerde")]
    pub position: Coord<f64>,
    /// The velocity of the agent in two dimensions
    pub velocity: ConstVel2D,
    /// The safety distance in the x-axis direction
    pub safety_x: f64,
    /// The order of the agent
    pub order: i64,
}

/// Motion of constant velocity in two dimensions.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ConstVel2D {
    /// The velocity in the x-axis direction
    pub x: f64,
    /// The velocity in the y-axis direction
    pub y: f64,
}

/// The type of an action
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActionType {
    /// A scheduled action
    Scheduled,
    /// An evasive action
    Evasive,
    /// An idle action
    Idle,
}

/// An action is an event that is executed by an agent at a given location.
#[derive(Clone, Debug)]
pub struct Action {
    /// The agent executing the action
    pub agent: Agent,
    /// The target location of the action
    pub target: Coord,
    /// The duration of the action
    pub duration: f64,
    /// The type of the action
    pub r#type: ActionType,
}

impl Agent {
    /// Calculates the safety distance in the x-axis direction between this agent and another agent.
    pub fn safety_x(&self, other: &Agent) -> f64 {
        f64::max(self.safety_x, other.safety_x)
    }
}

/// A schedule is a list of events, determining the absolute order in which
/// they have to be executed.
#[derive(Debug)]
pub struct Schedule {
    /// The list of actions in the schedule
    pub actions: Vec<Action>,
}

/// A segment of a path
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Segment {
    /// The starting coordinate of the segment
    pub start: Coord,
    /// The ending coordinate of the segment
    pub end: Coord,
    /// The duration of the segment
    pub duration: f64,
}

/// A path is a list of moves necessary to arrive at the given action.
#[derive(Clone)]
pub struct Path {
    /// The list of moves that take the agent from the previous action to the current action
    pub moves: Vec<Segment>,
    /// The action to be performed
    pub action: Action,
    /// The start time of the path, which equals the end time of the previous path
    pub t_start: f64,
    /// The end time of the path, which is after the present action is finished
    pub t_end: f64,
}

/// A point in 3D space-time
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct PointST {
    /// The x-coordinate of the point
    pub x: f64,
    /// The y-coordinate of the point
    pub y: f64,
    /// The time of the point
    pub t: f64,
}

/// Serializer and deserializer for `Coord` type
#[derive(Serialize, Deserialize, Clone)]
#[serde(remote = "Coord")]
struct CoordSerde<T = f64>
where
    T: CoordNum,
{
    /// The x-coordinate of the coordinate
    x: T,
    /// The y-coordinate of the coordinate
    y: T,
}

/// Serializer and deserializer for `Coord` type used in `Polygon`
#[derive(Serialize, Deserialize, Clone)]
struct CoordSerdeForPolygon<T = f64>
where
    T: CoordNum,
{
    /// The x-coordinate of the coordinate
    x: T,
    /// The y-coordinate of the coordinate
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

/// Serializer and deserializer for `Polygon` type
#[derive(Serialize, Deserialize)]
#[serde(remote = "Polygon")]
struct PolygonSerde<T = f64>
where
    T: CoordNum,
{
    /// The exterior ring of the polygon
    #[serde(getter = "PolygonSerde::get_exterior")]
    exterior: Vec<CoordSerdeForPolygon<T>>,
    /// The interior rings of the polygon
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
