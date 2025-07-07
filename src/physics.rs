mod data;
mod model;

use data::Data;
use model::Model;

pub use rusty_mujoco as binding;
pub use binding::{ObjectId, obj};

use binding::{Obj, mjtObj};
use crate::error::Error;
use rustc_hash::FxHashMap;

pub struct Physics {
    model: Model,
    data: Data,
}

impl Physics {
    pub fn from_xml(xml_path: impl AsRef<std::path::Path>) -> Result<Self, Error> {
        let model = binding::mj_loadXML(xml_path.as_ref().to_str().unwrap())?;
        let data = binding::mj_makeData(&model);
        Ok(Self {
            model: Model { binding: model },
            data: Data { binding: data },
        })
    }

    pub fn from_xml_string(xml_string: impl Into<String>) -> Result<Self, Error> {
        let mut spec = binding::mj_parseXMLString(xml_string.into())?;
        let model = binding::mj_compile(&mut spec)
            .ok_or_else(|| Error::Mjs(binding::mjs_getError(&mut spec).unwrap_or_else(String::new)))?;
        let data = binding::mj_makeData(&model);
        Ok(Self {
            model: Model { binding: model },
            data: Data { binding: data },
        })
    }

    pub fn model(&self) -> &Model {
        &self.model
    }

    pub fn data(&self) -> &Data {
        &self.data
    }
    pub fn data_mut(&mut self) -> &mut Data {
        &mut self.data
    }

    pub fn step(&mut self) {
        rusty_mujoco::mj_step(&self.model.binding, &mut self.data.binding);
    }

    pub fn forward(&mut self) {
        rusty_mujoco::mj_forward(&self.model.binding, &mut self.data.binding);
    }

    pub fn reset(&mut self) {
        rusty_mujoco::mj_resetData(&self.model.binding, &mut self.data.binding);
    }

    pub fn object_id_of<O: Obj>(&self, name: &str) -> Option<ObjectId<O>> {
        binding::mj_name2id::<O>(&self.model.binding, name)
    }

    pub fn object_name_of<O: Obj>(&self, id: ObjectId<O>) -> String {
        binding::mj_id2name::<O>(&self.model.binding, id)
    }

    pub fn object_count_of<O: Obj>(&self) -> usize {
        match O::TYPE {
            mjtObj::BODY => self.model.binding.nbody(),
            mjtObj::JOINT => self.model.binding.njnt(),
            mjtObj::GEOM => self.model.binding.ngeom(),
            mjtObj::SITE => self.model.binding.nsite(),
            mjtObj::CAMERA => self.model.binding.ncam(),
            mjtObj::LIGHT => self.model.binding.nlight(),
            mjtObj::TENDON => self.model.binding.ntendon(),
            mjtObj::ACTUATOR => self.model.binding.nu(),
            _ => 0,
        }
    }
}

pub struct Actuators<'a> {
    physics: &'a mut Physics,
    name_to_id: FxHashMap<&'static str, ObjectId<binding::obj::Actuator>>,
}

impl Physics {
    pub fn actuators(&mut self) -> Actuators<'_> {
        Actuators {
            physics: self,
            name_to_id: FxHashMap::default(),
        }
    }
}

impl<'a> Actuators<'a> {
    pub fn set(&mut self, name: &'static str, control: f64) -> Result<(), Error> {
        let id = match self.name_to_id.get(name) {
            Some(&id) => id,
            None => {
                let id = self.physics.object_id_of::<binding::obj::Actuator>(name)
                    .ok_or(Error::NameNotFound(name))?;
                self.name_to_id.insert(name, id);
                id
            }
        };

        let Physics { model, data } = &mut self.physics;

        // TODO: provide safe ways to these (model, data) -> slice methods
        // via the `Physics`, that has a set of them
        (unsafe { data.ctrl_mut(&model) })[id.index()] = control;

        Ok(())
    }
}

/* TODO:
remove `type Qpos` and use `[f64; J::QPOS_SIZE]`, and the same for qvel,
when `generic_const_exprs` language feature is stabilzed */
#[allow(private_bounds)]
pub trait JointType: joint::Sealed {
    const MJT: binding::bindgen::mjtJoint;
    const QPOS_SIZE: usize;
    const QVEL_SIZE: usize;
    type Qpos: for<'q> TryFrom<&'q [f64]> + AsRef<[f64]>;
    type Qvel: for<'q> TryFrom<&'q [f64]> + AsRef<[f64]>;
}
pub mod joint {
    pub(super) trait Sealed {}

    pub struct Free;
    impl Sealed for Free {}
    impl super::JointType for Free {
        const MJT: super::binding::bindgen::mjtJoint = super::binding::bindgen::mjtJoint::FREE;
        const QPOS_SIZE: usize = 7; // x, y, z, q, qw, qx, qy, qz
        const QVEL_SIZE: usize = 6; // vx, vy, vz, wx, wy, wz
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }

    pub struct Ball;
    impl Sealed for Ball {}
    impl super::JointType for Ball {
        const MJT: super::binding::bindgen::mjtJoint = super::binding::bindgen::mjtJoint::BALL;
        const QPOS_SIZE: usize = 4; // qw, qx, qy, qz
        const QVEL_SIZE: usize = 3; // ωx, ωy, ωz
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }

    pub struct Hinge;
    impl Sealed for Hinge {}
    impl super::JointType for Hinge {
        const MJT: super::binding::bindgen::mjtJoint = super::binding::bindgen::mjtJoint::HINGE;
        const QPOS_SIZE: usize = 1; // angle [rad]
        const QVEL_SIZE: usize = 1; // angular_velocity
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }

    pub struct Slide;
    impl Sealed for Slide {}
    impl super::JointType for Slide {
        const MJT: super::binding::bindgen::mjtJoint = super::binding::bindgen::mjtJoint::SLIDE;
        const QPOS_SIZE: usize = 1; // position [m]
        const QVEL_SIZE: usize = 1; // linear_velocity
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }
}

impl Physics {
    pub fn time(&self) -> f64 {
        self.data.binding.time()
    }
    pub fn set_time(&mut self, time: f64) {
        self.data.binding.set_time(time);
    }

    pub fn qpos_of<J: JointType>(&self, id: ObjectId<obj::Joint>) -> Result<J::Qpos, Error> {
        let jnt_type = unsafe { std::mem::transmute(self.model.jnt_type()[id.index()]) };
        if jnt_type == J::MJT {
            let raw_qpos = unsafe { self.data.qpos(self.model()) };
            let offset = self.model.jnt_qposadr()[id.index()] as usize;
            Ok(J::Qpos::try_from(&raw_qpos[offset..(offset + J::QPOS_SIZE)]).ok().unwrap())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }
    pub fn set_qpos_of<J: JointType>(&mut self, id: ObjectId<obj::Joint>, qpos: J::Qpos) -> Result<(), Error> {
        // TODO: provide a way cast enum like `mjtJoint` to integer primitive
        let jnt_type = unsafe { std::mem::transmute(self.model.jnt_type()[id.index()]) };
        if jnt_type == J::MJT {
            let Self { model, data } = self;
            let raw_qpos = unsafe { data.qpos_mut(model) };
            let offset = model.jnt_qposadr()[id.index()] as usize;
            raw_qpos[offset..(offset + J::QPOS_SIZE)].copy_from_slice(qpos.as_ref());
            Ok(())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }

    pub fn control(&mut self) -> &mut [f64] {
        let Self { model, data } = self;
        unsafe { data.ctrl_mut(&model) }
    }
}
