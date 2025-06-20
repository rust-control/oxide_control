use std::ops::{Deref, DerefMut};
use rusty_mujoco::{MjModel, MjData, ObjectId, Obj};

pub trait Physics: Deref<Target = PhsyicsBase> + DerefMut<Target = PhsyicsBase> {
    fn derive(base: PhsyicsBase) -> Self;
}

pub struct PhsyicsBase {
    pub model: MjModel,
    pub data: MjData,
}

impl PhsyicsBase {
    pub(crate) fn qvel_index<O: Obj>(&self, obj: ObjectId<O>) -> Option<usize> {
        todo!()
    }
    pub(crate) fn qvel_size(&self, obj: ObjectId<impl Obj>) -> Option<usize> {
        

        todo!()
    }
}

impl PhsyicsBase {
    pub fn forward(&mut self) {
        rusty_mujoco::mj_forward(&self.model, &mut self.data);
    }

    /// 
    pub fn set_position<O: Obj>(&mut self, obj: ObjectId<O>, position: &[f64]) {
        todo!()
    }
}
