use crate::physics::Physics;

pub trait Environment {
    type Observation;
    type Action: Action<Environment = Self>;

    fn reset(&mut self) -> TimeStep<Self::Observation>;
    fn step(&mut self, action: &Self::Action) -> TimeStep<Self::Observation>;
    fn action_spec(&self) -> &dyn Space<Sample = Self::Action>;
    fn observation_spec(&self) -> &dyn Space<Sample = Self::Observation>;
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

pub enum Shape {
    Scaler,
    Vector(usize),
    Matrix(usize, usize),
    Tensor(usize, usize, usize),
}

/// Currently the data type is fixed to `f64`.
pub trait Space {
    /// The type of the sample in this space.
    type Sample;

    /// The shape of the sample in this space.
    fn shape(&self) -> Shape;
}

/////////////////////////////////////////////////////////////////

pub struct Composer(ComposedEnvironment);

pub struct ComposedEnvironment {
    physics: Physics,
    task: Option<Box<dyn Task>>,
    rewarders: Vec<Box<dyn Rewarder>>,
    observables: Vec<Box<dyn Observable>>,
}

/// ## Default
/// 
/// - `initialize_episode`: does nothing
/// - `should_terminate`: always returns `false`, never terminates the episode
pub trait Task {
    fn initialize_episode(&self, physics: &mut Physics) {
        // do nothing by default
    }

    fn should_terminate(&self, physics: &Physics) -> bool {
        false
    }
}

pub trait Rewarder {
    fn give(&self, physics: &Physics) -> f64;
}

pub trait Observable {
    fn observe(&self, physics: &Physics) -> Vec<f64>;
}

impl Composer {
    pub fn new(physics: Physics) -> Self {
        Self(ComposedEnvironment {
            physics,
            task: None,
            rewarders: Vec::new(),
            observables: Vec::new(),
        })
    }

    pub fn task(mut self, task: impl Task + 'static) -> Self {
        self.0.task = Some(Box::new(task));
        self
    }

    pub fn add_rewarder(mut self, rewarder: impl Rewarder + 'static) -> Self {
        self.0.rewarders.push(Box::new(rewarder));
        self
    }

    pub fn add_observable(mut self, observable: impl Observable + 'static) -> Self {
        self.0.observables.push(Box::new(observable));
        self
    }
}

impl Environment for ComposedEnvironment {

}

