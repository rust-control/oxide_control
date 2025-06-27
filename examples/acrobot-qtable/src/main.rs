use qtable::{QTable, QUpdate, strategy};

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

        let (elbow_id, shoulder_id) = (
            physycs.model.object_id_of(mujoco::ObjectType::mjOBJ_JOINT, "elbow").unwrap(),
            physycs.model.object_id_of(mujoco::ObjectType::mjOBJ_JOINT, "shoulder").unwrap(),
        );
        if self.do_swing {
            physycs.set_position(elbow_id, [rand::rng().random_range(-(PI/2.)..(PI/2.))]).unwrap();
        } else {
            physycs.set_position(elbow_id, [rand::rng().random_range(-(PI/10.)..(PI/10.))]).unwrap();
        }
        physycs.set_position(shoulder_id, [0.0]).unwrap();
    }

    fn should_finish_episode(&self, physics: &Self::Physics) -> bool {
    }

    fn get_reward(&self, physics: &Self::Physics) -> f64 {
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

    fn get_action_by_explore(
        &self,
        observation: &Observation,
    )
}

fn main() {
    println!("Hello, world!");
}
