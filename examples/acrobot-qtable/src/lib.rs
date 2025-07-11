mod np;

use qtable::{QTable, QConfig, QUpdate};
use oxide_control::physics::{ObjectId, obj, joint};
use std::f64::consts::PI;

pub struct Acrobot {
    raw: oxide_control::RawPhysics,
    elbow_id: ObjectId<obj::Joint>,
    shoulder_id: ObjectId<obj::Joint>,
    actuator_id: ObjectId<obj::Actuator>,
}
impl Acrobot {
    pub fn new() -> Self {
        let raw = oxide_control::RawPhysics::from_xml("acrobot.xml").unwrap();
        let elbow_id = raw.object_id_of::<obj::Joint>("elbow").unwrap();
        let shoulder_id = raw.object_id_of::<obj::Joint>("shoulder").unwrap();
        let actuator_id = raw.object_id_of::<obj::Actuator>("shoulder").unwrap();
        Self { raw, elbow_id, shoulder_id, actuator_id }
    }

    pub fn show_qpos(&self) {
        let elbow_qpos = self.qpos::<joint::Hinge>(self.elbow_id).unwrap();
        let shoulder_qpos = self.qpos::<joint::Hinge>(self.shoulder_id).unwrap();
        println!("Elbow QPos: {:?}, Shoulder QPos: {:?}", elbow_qpos, shoulder_qpos);
    }
    pub fn show_qvel(&self) {
        let elbow_qvel = self.qvel::<joint::Hinge>(self.elbow_id).unwrap();
        let shoulder_qvel = self.qvel::<joint::Hinge>(self.shoulder_id).unwrap();
        println!("Elbow QVel: {:?}, Shoulder QVel: {:?}", elbow_qvel, shoulder_qvel);
    }
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

pub struct Orientation {
    sin: f64,
    cos: f64,
}
impl Orientation {
    pub fn from_rad(rad: f64) -> Self {
        Self { sin: rad.sin(), cos: rad.cos() }
    }

    pub fn to_rad(&self) -> f64 {
        f64::atan2(self.sin, self.cos)
    }
}

pub struct AcrobotObservation {
    pub elbow_orientation: Orientation,
    pub shoulder_orientation: Orientation,
    pub elbow_velocity: f64,
    pub shoulder_velocity: f64,
}
impl oxide_control::Observation for AcrobotObservation {
    type Physics = Acrobot;

    fn generate(physics: &Self::Physics) -> Self {
        let [elbow_rad] = physics.qpos::<joint::Hinge>(physics.elbow_id).unwrap();
        let [shoulder_rad] = physics.qpos::<joint::Hinge>(physics.shoulder_id).unwrap();
        let [elbow_velocity] = physics.qvel::<joint::Hinge>(physics.elbow_id).unwrap();
        let [shoulder_velocity] = physics.qvel::<joint::Hinge>(physics.shoulder_id).unwrap();
        Self {
            elbow_orientation: Orientation::from_rad(elbow_rad),
            shoulder_orientation: Orientation::from_rad(shoulder_rad),
            elbow_velocity,
            shoulder_velocity,            
        }
    }
}

pub struct BalanceTask {
    pub do_swing: bool,
    pub discount: f64,
    pub n_arm_digitization: usize,
    pub n_pendulum_digitization: usize,
    pub arm_limit: std::ops::Range<f64>,
    pub pendulum_limit: std::ops::Range<f64>,
}
#[derive(Clone, Copy)]
#[allow(unused)]
pub struct AcrobotState {
    pub arm_rad: f64,
    pub pendulum_rad: f64,
    pub arm_vel: f64,
    pub pendulum_vel: f64,
    pub n_arm_rad: usize,
    pub n_pendulum_rad: usize,
    pub n_arm_vel: usize,
    pub n_pendulum_vel: usize,
    pub digitized_state: usize,
}
impl BalanceTask {
    pub fn state(&self, observation: &AcrobotObservation) -> AcrobotState {
        let d_arm = self.n_arm_digitization;
        let d_pendulum = self.n_pendulum_digitization;
        let arm_rad = observation.shoulder_orientation.to_rad();
        let arm_vel = observation.shoulder_velocity;
        let pendulum_rad = observation.elbow_orientation.to_rad();
        let pendulum_vel = observation.elbow_velocity;

        let n_arm_rad = np::digitize(arm_rad, &np::linspace(self.arm_limit.start, self.arm_limit.end, d_arm + 1)[1..d_arm]);
        let n_arm_vel = np::digitize(arm_vel.clamp(-8.0, 8.0), &np::linspace(-8.0, 8.0, d_arm + 1)[1..d_arm]);
        let n_pendulum_rad = np::digitize(pendulum_rad, &np::linspace(self.pendulum_limit.start, self.pendulum_limit.end, d_pendulum + 1)[1..d_pendulum]);
        let n_pendulum_vel = np::digitize(pendulum_vel.clamp(-8.0, 8.0), &np::linspace(-8.0, 8.0, d_pendulum + 1)[1..d_pendulum]);

        let digitized_state =
            n_pendulum_rad +
            n_pendulum_vel * d_pendulum +
            n_arm_vel * d_pendulum.pow(2) +
            n_arm_rad * d_pendulum.pow(2) * d_arm;

        AcrobotState {
            arm_rad,
            pendulum_rad,
            arm_vel,
            pendulum_vel,
            n_arm_rad,
            n_pendulum_rad,
            n_arm_vel,
            n_pendulum_vel,
            digitized_state,
        }
    }
}
impl oxide_control::Task for BalanceTask {
    type Physics = Acrobot;
    
    type Observation = AcrobotObservation;

    fn discount(&self) -> f64 {
        self.discount
    }

    fn init_episode(&self, physics: &mut Self::Physics) {
        let (elbow_id, shoulder_id) = (physics.elbow_id, physics.shoulder_id);
        let elbow_qpos_range = if self.do_swing {-(PI/2.)..(PI/2.)} else {-(PI/10.)..(PI/10.)};
        physics.set_qpos::<joint::Hinge>(elbow_id, [rand::Rng::random_range(&mut rand::rng(), elbow_qpos_range)]).unwrap();
        physics.set_qpos::<joint::Hinge>(shoulder_id, [0.]).unwrap();
    }

    fn should_finish_episode(&self, observation: &Self::Observation, _: &Self::Physics) -> bool {
        let state = self.state(observation);
        if state.n_arm_rad <= 0 || self.n_arm_digitization <= state.n_arm_rad {
            return true; // Arm is out of bounds
        }
        if state.n_pendulum_rad <= 0 || self.n_pendulum_digitization <= state.n_pendulum_rad {
            return true; // Pendulum is out of bounds
        }
        false
    }

    fn get_reward(&self, observation: &Self::Observation, physics: &Self::Physics) -> f64 {
        let state = self.state(observation);

        if self.should_finish_episode(observation, physics) {
            return -10.0; // Penalty for finishing the episode
        }
        
        let best_n_pendulum_rad = self.n_pendulum_digitization / 2;
        let good_n_pendulum_rad_min = best_n_pendulum_rad - (self.n_pendulum_digitization / 4);
        let good_n_pendulum_rad_max = best_n_pendulum_rad + (self.n_pendulum_digitization / 4);

        let best_n_arm_rad = self.n_arm_digitization / 2;
        let good_n_arm_rad_min = best_n_arm_rad - (self.n_arm_digitization / 4);
        let good_n_arm_rad_max = best_n_arm_rad + (self.n_arm_digitization / 4);

        let (n_pend_rad, n_arm_rad) = (state.n_pendulum_rad, state.n_arm_rad);
        let pend_reward = if n_pend_rad == best_n_pendulum_rad {
            2.0
        } else if (best_n_pendulum_rad - 1..=best_n_pendulum_rad + 1).contains(&n_pend_rad) {
            1.5
        } else if (good_n_pendulum_rad_min..=good_n_pendulum_rad_max).contains(&n_pend_rad) {
            1.0 - (n_pend_rad as f64 - best_n_pendulum_rad as f64).abs() / (good_n_pendulum_rad_max - best_n_pendulum_rad) as f64
        } else {
            - (n_pend_rad as f64 - best_n_pendulum_rad as f64).abs() / (self.n_pendulum_digitization as f64) * 2.0
        };
        let arm_reward = if n_arm_rad == best_n_arm_rad {
            2.0
        } else if (best_n_arm_rad - 1..=best_n_arm_rad + 1).contains(&n_arm_rad) {
            1.5
        } else if (good_n_arm_rad_min..=good_n_arm_rad_max).contains(&n_arm_rad) {
            1.0 - (n_arm_rad as f64 - best_n_arm_rad as f64).abs() / (good_n_arm_rad_max - best_n_arm_rad) as f64
        } else {
            - (n_arm_rad as f64 - best_n_arm_rad as f64).abs() / (self.n_arm_digitization as f64) * 2.0
        };
        pend_reward + arm_reward
    }
}

#[derive(Clone, Copy)]
pub struct AcrobotAction {
    actuator_id: ObjectId<obj::Actuator>,
    digitization_index: usize,
    torque: f64,
}
impl oxide_control::Action for AcrobotAction {
    type Physics = Acrobot;
    
    fn apply(self, actuators: &mut oxide_control::physics::Actuators<'_>) {
        assert!(!self.torque.is_nan(), "Torque cannot be NaN");
        assert!((-1.0..=1.0).contains(&self.torque), "Torque must be in the range [-1.0, 1.0]");
        actuators.set(self.actuator_id, self.torque);
    }
}

pub struct Agent {
    qtable: QTable,
    digitized_actions: Vec<AcrobotAction>,
}
pub struct AgentConfig {
    pub action_size: usize,
    pub state_size: usize,
}
impl Agent {
    fn make_digitized_actions(physics: &Acrobot, config: &AgentConfig) -> Vec<AcrobotAction> {
        let ctrlrange = physics.model().actuator_ctrlrange(physics.actuator_id);
        assert_eq!(ctrlrange, -1.0..1.0, "The control range must be [-1.0, 1.0] for the Acrobot model.");
        let digitized_torques = np::linspace(ctrlrange.start, ctrlrange.end, config.action_size);
        digitized_torques
            .into_iter()
            .enumerate()
            .map(|(index, torque)| AcrobotAction {
                actuator_id: physics.actuator_id,
                digitization_index: index,
                torque,
            })
            .collect::<Vec<_>>()
    }

    pub fn new(physics: &Acrobot, config: &AgentConfig) -> Self {
        let qtable = QTable::new_with(QConfig {
            action_size: config.action_size,
            state_size: config.state_size,
            ..Default::default()
        });
        let digitized_actions = Self::make_digitized_actions(physics, config);
        Self { qtable, digitized_actions }
    }

    pub fn new_with_resotoring_qtable(
        qtable_filename: impl AsRef<std::path::Path>,
        physics: &Acrobot,
        config: &AgentConfig,
    ) -> Result<Self, std::io::Error> {
        let qtable = QTable::load_with(qtable_filename, QConfig {
            action_size: config.action_size,
            state_size: config.state_size,
            ..Default::default()
        })?;
        let digitized_actions = Self::make_digitized_actions(physics, config);
        Ok(Self { qtable, digitized_actions })
    }

    pub fn get_action<S: qtable::Strategy>(
        &self,
        state: AcrobotState,
    ) -> AcrobotAction {
        let qtable_state = qtable::State::new_on(&self.qtable, state.digitized_state).unwrap();
        let qtable_action = self.qtable.next_action::<S>(qtable_state);
        self.digitized_actions[qtable_action.index()]
    }

    pub fn learn(
        &mut self,
        state: AcrobotState,
        action: AcrobotAction,
        reward: f64,
        next_state: AcrobotState,
    ) {
        self.qtable.update(QUpdate {
            state: qtable::State::new_on(&self.qtable, state.digitized_state).unwrap(),
            action: qtable::Action::new_on(&self.qtable, action.digitization_index).unwrap(),
            reward,
            next_state: qtable::State::new_on(&self.qtable, next_state.digitized_state).unwrap(),
        });
    }

    pub fn save_qtable(&self, file_path: impl AsRef<std::path::Path>) -> Result<(), std::io::Error> {
        self.qtable.save(file_path)
    }
}
