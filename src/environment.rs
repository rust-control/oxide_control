use crate::physics::Physics;

pub trait Environment {
    type Observation;
    type Action: Action<Environment = Self>;

    fn reset(&mut self) -> TimeStep<Self::Observation>;
    fn step(&mut self, action: &Self::Action) -> TimeStep<Self::Observation>;
    fn action_spec(&self) -> ArraySpec;
    fn observation_spec(&self) -> ArraySpec;
}

pub trait Action {
    type Environment: Environment;
}

pub struct TimeStep<O> {
    pub step_type: StepType,
    pub reward: f32,
    pub discount: f32,
    pub observation: O,
}

pub enum StepType {
    First,
    Mid,
    Last,
}

pub struct ArraySpec {
    /// Currently not used
    pub dtype: DType,
    pub shape: Shape,
}
/// Currently not used
pub enum DType {
    F64,
    I32,
    U8,
}
pub enum Shape {
    Scaler,
    Vector(usize),
    Matrix(usize, usize),
    Tensor(usize, usize, usize),
}

// /// Currently the data type is fixed to `f64`.
// pub trait Space {
//     /// The type of the sample in this space.
//     type Sample;
// 
//     /// The shape of this space.
//     fn shape(&self) -> Shape;
// }

/////////////////////////////////////////////////////////////////

pub struct Composer<O = ()>(ComposedEnvironment<O>);

pub struct ComposedEnvironment<O> {
    physics: Physics,
    observer: Option<Box<dyn Observer<Observation = O>>>,
    task: Option<Box<dyn Task>>,
    rewarders: Vec<Box<dyn Rewarder>>,
}

pub trait Observer {
    type Observation;
    fn observe(&self, physics: &Physics) -> Self::Observation;
}

/// A task defines the goal of the environment and how to determine if the task is complete.
/// 
/// Rewards are speparated from tasks to allow for more flexible reward structures.
/// See [`Rewarder`] and [`Composer`].
/// 
/// ### required
/// 
/// - `should_terminate(&self, Physics) -> bool`: Determines if the task is complete
pub trait Task {
    fn initialize_episode(&self, #[allow(unused)] physics: &mut Physics) {
        // do nothing by default
    }

    fn should_terminate(&self, physics: &Physics) -> bool;
}

pub trait Rewarder {
    fn give(&self, physics: &Physics) -> f64;
}

impl Composer {
    pub fn new(physics: Physics) -> Self {
        Self(ComposedEnvironment {
            physics,
            observer: None,
            task: None,
            rewarders: Vec::new(),
        })
    }

    pub fn observer<O>(self, observer: impl Observer<Observation = O> + 'static) -> Composer<O> {
        Composer(ComposedEnvironment {
            observer: Some(Box::new(observer)),
            physics: self.0.physics,
            task: self.0.task,
            rewarders: self.0.rewarders,
        })
    }
}

impl<O> Composer<O> {
    pub fn task(mut self, task: impl Task + 'static) -> Self {
        self.0.task = Some(Box::new(task));
        self
    }

    pub fn add_rewarder(mut self, rewarder: impl Rewarder + 'static) -> Self {
        self.0.rewarders.push(Box::new(rewarder));
        self
    }

    /// Composes the final environment.
    pub fn compose(self) -> impl Environment {
        self.0
    }
}

impl<O, A: Action<Environment = ComposedEnvironment<O>>> Environment for ComposedEnvironment<O> {
    type Observation = O;
    type Action = A;

}

