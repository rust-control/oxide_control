use std::ops::{Deref, DerefMut};
use rusty_mujoco::{MjModel, MjData};

pub trait Physics: Deref<Target = PhsyicsBase> + DerefMut<Target = PhsyicsBase> {
    fn derive(base: PhsyicsBase) -> Self;
}

pub struct PhsyicsBase {
    pub model: MjModel,
    pub data: MjData,
}
