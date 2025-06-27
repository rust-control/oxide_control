use qtable::{QTable, QUpdate, strategy};
use oxide_control::physics::binding::obj;

struct Acrobot(oxide_control::RawPhysics);

impl std::ops::Deref for Acrobot {
    type Target = oxide_control::RawPhysics;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for Acrobot {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl oxide_control::Physics for Acrobot {}

struct AcrobotObservation {
    elbow_orientation: Orientation,
    shoulder_orientation: Orientation,
    
}
struct Orientation {
    sin: f64,
    cos: f64,
}

struct BalanceTask {
    do_swing: bool,
    discount: f64,
}

impl oxide_control::Task for BalanceTask {
    type Physics = Acrobot;

    fn discount(&self) -> f64 {
        self.discount
    }

    fn init_episode(&self, physics: &mut Self::Physics) {
        use std::f64::consts::PI;
        use rand::Rng;

        let elbow_id = physics.model().object_id_of::<obj::Joint>("elbow").unwrap();
        let shoulder_id = physics.model().object_id_of::<obj::Joint>("shoulder").unwrap();

        let mut rng = rand::rng();
        let qpos_mut = unsafe {// TODO: prepare a better way to access and mutate `qpos` and others like `qvel`
            let nq = physics.model().nq();
            physics.data_mut().qpos_mut(nq)
        };
        
        // TODO: check each joint's qpos element size
        // (here we already know `elbow` and `shoulder` are both 1 element)
        let elbow_qpos_range = if self.do_swing {-(PI/2.)..(PI/2.)} else {-(PI/10.)..(PI/10.)};
        qpos_mut[elbow_id.index()] = rng.random_range(elbow_qpos_range);
        qpos_mut[shoulder_id.index()] = 0.;
    }

    fn should_finish_episode(&self, physics: &Self::Physics) -> bool {
        false // physics.get_position(shoulder_id).unwrap() > 0.2
    }

    fn get_reward(&self, physics: &Self::Physics) -> f64 {
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
