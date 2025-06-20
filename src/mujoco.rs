pub use rusty_mujoco as binding;

pub use binding::{ObjectId, ObjType, Obj};

pub struct Model(binding::MjModel);

impl Model {
    pub fn object_id_of<O: Obj>(&self, name: &str) -> Option<ObjectId<O>> {
        binding::mj_name2id::<O>(&self.0, name)
    }

    pub fn object_name_of<O: Obj>(&self, id: ObjectId<O>) -> String {
        binding::mj_id2name::<O>(&self.0, id)
    }

    pub fn object_count_of<O: Obj>(&self) -> usize {
        match O::TYPE {
            ObjType::Body => self.0.nbody(),
            ObjType::Joint => self.0.njnt(),
            ObjType::Geom => self.0.ngeom(),
            ObjType::Site => self.0.nsite(),
            ObjType::Camera => self.0.ncam(),
            ObjType::Light => self.0.nlight(),
            ObjType::Tendon => self.0.ntendon(),
            ObjType::Actuator => self.0.na(),
            _ => 0,
        }
    }
}

pub struct Data(binding::MjData);
