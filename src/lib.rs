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
    fn apply(self, actuators: &mut physics::Actuators<'_>);
}

pub struct Environment<T, A>
where
    T: Task,
    A: Action<Physics = T::Physics>,
{
    task: T,
    physics: T::Physics,
    _a: std::marker::PhantomData<A>,
}

const _: (/* for ease with using `Environment::new(...)` */) = {
    pub struct DummyAction<P: Physics>(std::marker::PhantomData<P>);
    impl<P: Physics> Action for DummyAction<P> {
        type Physics = P;
        fn apply(self, _actuators: &mut physics::Actuators<'_>) {
            // No action to apply
        }
    }
    
    impl<T: Task> Environment<T, DummyAction<T::Physics>> {
        pub fn new<A: Action<Physics = T::Physics>>(physics: T::Physics, task: T) -> Environment<T, A> {
            Environment {
                task,
                physics,
                _a: std::marker::PhantomData,
            }
        }
    }
};

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

impl<T, A> Environment<T, A>
where
    T: Task,
    A: Action<Physics = T::Physics>,
{
    pub fn reset(&mut self) -> T::Observation {
        self.task.init_episode(&mut self.physics);
        T::Observation::generate(&self.physics)
    }

    pub fn step(&mut self, action: A) -> TimeStep<T::Observation> {
        action.apply(&mut self.physics.actuators());
        self.physics.step();

        let observation = T::Observation::generate(&self.physics);
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
