mod np;

use qtable::{QTable, QUpdate, strategy};
use oxide_control::physics::{ObjectId, obj, joint};
use std::f64::consts::PI;

struct Acrobot {
    raw: oxide_control::RawPhysics,
    elbow_id: ObjectId<obj::Joint>,
    shoulder_id: ObjectId<obj::Joint>,
}
impl Acrobot {
    fn new() -> Self {
        let raw = oxide_control::RawPhysics::from_xml("acrobot.xml").unwrap();
        let elbow_id = raw.object_id_of("elbow").unwrap();
        let shoulder_id = raw.object_id_of("shoulder").unwrap();
        Self { raw, elbow_id, shoulder_id }
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

struct AcrobotObservation {
    elbow_orientation: Orientation,
    shoulder_orientation: Orientation,
    elbow_velocity: f64,
    shoulder_velocity: f64,
}

struct Orientation {
    sin: f64,
    cos: f64,
}
impl Orientation {
    fn from_rad(rad: f64) -> Self {
        Self { sin: rad.sin(), cos: rad.cos() }
    }

    fn to_rad(&self) -> f64 {
        f64::atan2(self.sin, self.cos)
    }
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

struct BalanceTask {
    do_swing: bool,
    discount: f64,
    n_arm_digitization: usize,
    n_pendulum_digitization: usize,
    arm_limit: std::ops::Range<f64>,
    pendulum_limit: std::ops::Range<f64>,
}

struct AcrobotState {
    arm_rad: f64,
    pendulum_rad: f64,
    arm_vel: f64,
    pendulum_vel: f64,
    n_arm_rad: usize,
    n_pendulum_rad: usize,
    n_arm_vel: usize,
    n_pendulum_vel: usize,
    digitized_state: usize,
}

impl BalanceTask {
    fn state(&self, observation: &AcrobotObservation) -> AcrobotState {
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
            n_arm_rad * d_pendulum.pow(2) * (d_arm);

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
        let elbow_qpos_range = if self.do_swing {-(PI/2.)..(PI/2.)} else {-(PI/10.)..(PI/10.)};
        let (elbow_id, shoulder_id) = (physics.elbow_id, physics.shoulder_id);
        physics.set_qpos::<joint::Hinge>(elbow_id, [rand::Rng::random_range(&mut rand::rng(), elbow_qpos_range)]).unwrap();
        physics.set_qpos::<joint::Hinge>(shoulder_id, [0.]).unwrap();
    }

    fn should_finish_episode(&self, observation: &Self::Observation, _: &Self::Physics) -> bool {
        let state_dict = self.state(observation);
        if !(0..=self.n_arm_digitization).contains(&state_dict.n_arm_rad) {
            return true; // Arm is out of bounds
        }
        if !(0..=self.n_pendulum_digitization).contains(&state_dict.n_pendulum_rad) {
            return true; // Pendulum is out of bounds
        }
        false
    }

    fn get_reward(&self, observation: &Self::Observation, physics: &Self::Physics) -> f64 {
        let state_dict = self.state(observation);

        if self.should_finish_episode(observation, physics) {
            return -10.0; // Penalty for finishing the episode
        }
        
        let best_n_pendulum_rad = (self.n_pendulum_digitization - 1) / 2;
        let good_n_pendulum_rad_min = best_n_pendulum_rad - (self.n_pendulum_digitization / 4);
        let good_n_pendulum_rad_max = best_n_pendulum_rad + (self.n_pendulum_digitization / 4);

        let n_pendulum_rad = state_dict.n_pendulum_rad;
        //                     |---------------|------------------|------------------|--------------------|
        // n_pendulum_rad:     0            good_min             best             good_max        n_pendulum_digitization
        // reward:         {negative}         0.0                1.0                0.0               {negative}
        // 
        // 
        // 1. We'll provide a reward of `y = - ax^2 + bx + c` form for the `n_pendulum_rad`.
        // 2. The rewrard must equal to 1.0 at `best_n_pendulum_rad`.
        // 3. The reward must be positive between `good_n_pendulum_rad_min` and `good_n_pendulum_rad_max`.
        // 4. The reward must be `0.0` at `good_n_pendulum_rad_min` and `good_n_pendulum_rad_max`.
        // 5. The reward must be negative in the rest of the range.
        // 
        // We can set up a system of equations to find the coefficients `a`, `b`, and `c`:
        // 
        //     let gn = good_n_pendulum_rad_min in
        //     let gx = good_n_pendulum_rad_max in
        //     let best = best_n_pendulum_rad in
        //     and(
        //         0 = -a*gn**2 + b*gn + c,
        //         1 = -a*best**2 + b*best + c,
        //         0 = -a*gx**2 + b*gx + c
        //     )
        // 
        // Solving this system of equations gives us:
        let (gn, gx, best) = (good_n_pendulum_rad_min as f64, good_n_pendulum_rad_max as f64, best_n_pendulum_rad as f64);
        let a = 1.0 / (best.powi(2) - gn * gx);
        let b = a * (gn + gx);
        let c = -a * gn * gx;
        // So, the reward is:
        let x = n_pendulum_rad as f64;
        -a * x.powi(2) + b * x + c
    }
}

struct AcrobotAction {
    actuator_id: ObjectId<obj::Actuator>,
    torque: f64,
}
impl AcrobotAction {
    fn new(torque: f64, physics: &Acrobot) -> Self {
        Self {
            actuator_id: physics.object_id_of("shoulder").unwrap(),
            torque,
        }
    }
}
impl oxide_control::Action for AcrobotAction {
    type Physics = Acrobot;
    
    fn apply(self, actuators: &mut oxide_control::physics::Actuators<'_>) {
        actuators.set(self.actuator_id, self.torque);
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

    fn learn(
        &mut self,
        state: AcrobotState,
        action: qtable::Action,
        reward: f64,
        next_state: AcrobotState,
    ) {
        self.qtable.update(QUpdate {
            state: qtable::State::new_on(&self.qtable, state.digitized_state).unwrap(),
            action,
            reward,
            next_state: qtable::State::new_on(&self.qtable, next_state.digitized_state).unwrap(),
        });
    }

    // fn get_action<S: qtable::Strategy>(
    //     &self,
    //     observation: &Observation
    // ) -> qtable::Action {
    //     
    // }
}

fn main() {
    let e = oxide_control::Environment::new::<AcrobotAction>(
        Acrobot::new(),
        BalanceTask {
            do_swing: false,
            discount: 0.99,
            n_arm_digitization: 10,
            n_pendulum_digitization: 10,
            arm_limit: -PI..PI,
            pendulum_limit: -PI..PI,
        },
    );
}
