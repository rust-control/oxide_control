use qtable::{QTable, QUpdate, strategy};
use oxide_control::physics::{ObjectId, obj, joint};

struct Acrobot {
    raw: oxide_control::RawPhysics,
    elbow: ObjectId<obj::Joint>,
    shoulder: ObjectId<obj::Joint>,
}

impl std::ops::Deref for Acrobot {
    type Target = oxide_control::RawPhysics;

    fn deref(&self) -> &Self::Target {
        &self.raw
    }
}
impl std::ops::DerefMut for Acrobot {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.raw
    }
}
impl oxide_control::Physics for Acrobot {}

struct AcrobotObservation {
    elbow_orientation: Orientation,
    shoulder_orientation: Orientation,
    velocity: (),
}
struct Orientation {
    sin: f64,
    cos: f64,
}
impl Orientation {
    fn from_rad(rad: f64) -> Self {
        Self { sin: rad.sin(), cos: rad.cos() }
    }
}

impl oxide_control::Observation for AcrobotObservation {
    type Physics = Acrobot;

    fn generate(physics: &Self::Physics) -> Self {
        let elbow_rad = physics.qpos::<joint::Hinge>(physics.elbow).unwrap();
        let shoulder_rad = physics.qpos::<joint::Hinge>(physics.shoulder).unwrap();
        Self {
            elbow_orientation: Orientation::from_rad(elbow_rad),
            shoulder_orientation: Orientation::from_rad(shoulder_rad),
            velocity: (), // TODO: fill in with actual velocity data
        }
    }
}

struct BalanceTask {
    do_swing: bool,
    discount: f64,
}

impl oxide_control::Task for BalanceTask {
    type Physics = Acrobot;
    
    type Observation = AcrobotObservation;

    fn discount(&self) -> f64 {
        self.discount
    }

    fn init_episode(&self, physics: &mut Self::Physics) {
        use std::f64::consts::PI;
        use rand::Rng;

        let elbow_qpos_range = if self.do_swing {-(PI/2.)..(PI/2.)} else {-(PI/10.)..(PI/10.)};
        physics.set_qpos::<joint::Hinge>(physics.elbow, [rng.random_range(elbow_qpos_range)]);
        physics.set_qpos::<joint::Hinge>(physics.shoulder, [0.]);
    }

    fn should_finish_episode(&self, observation: &Self::Observation, physics: &Self::Physics) -> bool {
        false // physics.get_position(shoulder_id).unwrap() > 0.2
    }

    fn get_reward(&self, observation: &Self::Observation, physics: &Self::Physics) -> f64 {
        todo!()
    }
}

struct Agent {
    qtable: QTable,
}

impl Agent {
    fn new() -> Self {
        Self {
            qtable: QTable::new(),
        }
    }

    // fn get_action_by_explore(
    //     &self,
    //     observation: &Observation,
    // )
}

fn main() {
    println!("Hello, world!");
}
