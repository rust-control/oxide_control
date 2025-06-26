pub trait Environment {
    type Observation;
    type Action;

    fn reset(&mut self) -> TimeStep<Self::Observation>;
    fn step(&mut self, action: &Self::Action) -> TimeStep<Self::Observation>;
    fn action_spec(&self) -> &dyn Space<Sample = Self::Action>;
    fn observation_spec(&self) -> &dyn Space<Sample = Self::Observation>;

}

pub struct TimeStep<O> {
    pub step_type: StepType, // First, Mid, Last
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
