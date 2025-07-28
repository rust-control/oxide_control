pub mod error;
pub mod physics;

pub use physics::Physics as RawPhysics;

pub trait Physics: std::ops::DerefMut<Target = RawPhysics> {}

pub trait Task {
    type Physics: Physics;
    type Observation: Observation<Physics = Self::Physics>;
    type Action: Action<Physics = Self::Physics>;
    fn discount(&self) -> f64;
    fn init_episode(&self, physics: &mut Self::Physics);
    fn should_finish_episode(&self, observation: &Self::Observation) -> bool;
    fn get_reward(&self, observation: &Self::Observation, action: &Self::Action) -> f64;
}

pub trait Observation {
    type Physics: Physics;
    fn generate(physics: &Self::Physics) -> Self;
}

pub trait Action {
    type Physics: Physics;
    fn apply(&self, actuators: &mut physics::Actuators<'_>);
}

pub struct Environment<T: Task> {
    task: T,
    physics: T::Physics,
}

impl<T: Task> Environment<T> {
    pub fn new(physics: T::Physics, task: T) -> Self {
        Self { task, physics }
    }

    pub fn task(&self) -> &T {
        &self.task
    }

    pub fn physics(&self) -> &T::Physics {
        &self.physics
    }
    pub fn physics_mut(&mut self) -> &mut T::Physics {
        &mut self.physics
    }
}

pub enum TimeStep<O> {
    Step {
        observation: O,
        reward: f64,
        discount: f64,
    },
    Finish {
        observation: O,
        reward: f64,
    },
}

impl<T: Task> Environment<T> {
    pub fn reset(&mut self) -> T::Observation {
        self.task.init_episode(&mut self.physics);
        T::Observation::generate(&self.physics)
    }

    pub fn step(&mut self, action: T::Action) -> TimeStep<T::Observation> {
        action.apply(&mut self.physics.actuators());
        self.physics.step();

        let observation = T::Observation::generate(&self.physics);
        let reward = self.task.get_reward(&observation, &action);

        if self.task.should_finish_episode(&observation) {
            TimeStep::Finish {
                observation,
                reward,
            }
        } else {
            TimeStep::Step {
                observation,
                reward,
                discount: self.task.discount(),
            }
        }
    }
}
