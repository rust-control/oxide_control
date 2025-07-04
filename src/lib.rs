pub mod error;
pub mod physics;

pub use physics::Physics as RawPhysics;

pub trait Physics: std::ops::DerefMut<Target = RawPhysics> {}

pub trait Task {
    type Physics: Physics;
    type Observation: Observation<Physics = Self::Physics>;
    fn discount(&self) -> f64;
    fn init_episode(&self, physics: &mut Self::Physics);
    fn should_finish_episode(
        &self,
        observation: &Self::Observation,
        physics: &Self::Physics,
    ) -> bool;
    fn get_reward(
        &self,
        observation: &Self::Observation,
        physics: &Self::Physics,
    ) -> f64;
}

pub trait Observation {
    type Physics: Physics;
    fn generate(physics: &Self::Physics) -> Self;
}

pub trait Action {
    type Physics: Physics;
    fn apply(self, actuators: physics::Actuators<'_>);
}

pub struct Environment<T, O, A>
where
    T: Task<Observation = O>,
    O: Observation<Physics = T::Physics>,
    A: Action<Physics = T::Physics>,
{
    task: T,
    physics: T::Physics,
    _o: std::marker::PhantomData<O>,
    _a: std::marker::PhantomData<A>,
}

impl<T, O, A> Environment<T, O, A>
where
    T: Task<Observation = O>,
    O: Observation<Physics = T::Physics>,
    A: Action<Physics = T::Physics>,
{
    pub fn new(physics: T::Physics, task: T) -> Self {
        Self {
            task,
            physics,
            _o: std::marker::PhantomData,
            _a: std::marker::PhantomData,
        }
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

impl<T, O, A> Environment<T, O, A>
where
    T: Task<Observation = O>,
    O: Observation<Physics = T::Physics>,
    A: Action<Physics = T::Physics>,
{
    pub fn reset(&mut self) -> O {
        self.task.init_episode(&mut self.physics);
        O::generate(&self.physics)
    }

    pub fn step(&mut self, action: A) -> TimeStep<O> {
        action.apply(self.physics.actuators());
        self.physics.step();

        let observation = O::generate(&self.physics);
        let reward = self.task.get_reward(&observation, &self.physics);
        
        if self.task.should_finish_episode(&observation, &self.physics) {
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
