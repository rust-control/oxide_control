pub mod error;
pub mod physics;

pub use physics::Physics as RawPhysics;

/* core interfaces */

pub trait Environment<T, O, A>
where
    T: Task,
    O: Observation<Physics = T::Physics>,
    A: Action<Physics = T::Physics>,
{
    fn reset(&mut self) -> O;
    fn step(&mut self, action: A) -> TimeStep<O>;
}

pub enum TimeStep<O: Observation> {
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

pub trait Phyics: std::ops::DerefMut<Target = RawPhysics> {}

pub trait Task {
    type Physics: Phyics;
    fn init_episode(&mut self, physics: &mut Self::Physics);
    fn should_finish_episode(&self, physics: &Self::Physics) -> bool;
    fn get_reward(&self, physics: &Self::Physics) -> f64;
    fn get_discount(&self, physics: &Self::Physics) -> f64;
}

pub trait Observation {
    type Physics: Phyics;
    fn generate(physics: &Self::Physics) -> Self;
}

pub trait Action {
    type Physics: Phyics;
    fn apply(self, physics: &mut Self::Physics);
}
