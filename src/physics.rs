mod data;
mod model;

use data::Data;
use model::Model;

pub use rusty_mujoco as binding;
pub use binding::{ObjectId, obj, mjMAXVAL, mjMINVAL};

use binding::{Obj, mjtObj};
use crate::error::Error;

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

    pub fn model_data(&self) -> (&Model, &Data) {
        (&self.model, &self.data)
    }
    pub fn model_datamut(&mut self) -> (&Model, &mut Data) {
        (&self.model, &mut self.data)
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
}

impl Physics {
    pub fn actuators(&mut self) -> Actuators<'_> {
        Actuators {
            physics: self,
        }
    }
}

impl<'a> Actuators<'a> {
    pub fn set(&mut self, id: ObjectId<obj::Actuator>, control: f64) {
        self.physics.set_ctrl(id, control);
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

    pub fn ctrl(&self, id: ObjectId<obj::Actuator>) -> f64 {
        let Self { model, data } = self;
        (unsafe { data.binding.ctrl(&model.binding) })[id.index()]
    }
    pub fn set_ctrl(&mut self, id: ObjectId<obj::Actuator>, value: f64) {
        let Self { model, data } = self;
        (unsafe { data.binding.ctrl_mut(&model.binding) })[id.index()] = value;
    }

    pub fn act(&self, id: ObjectId<obj::Actuator>) -> Option<f64> {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.act(&model.binding) };
        let offset = model.actuator_actadr(id)?;
        Some(slice[offset])
    }
    pub fn set_act(&mut self, id: ObjectId<obj::Actuator>, value: f64) -> Result<(), Error> {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.act_mut(&model.binding) };
        let offset = model.actuator_actadr(id).ok_or(Error::ActuatorStateless(id))?;
        slice[offset] = value;
        Ok(())
    }

    pub fn qpos<J: JointType>(&self, id: ObjectId<obj::Joint>) -> Result<J::Qpos, Error> {
        let Self { model, data } = self;
        let jnt_type = unsafe { std::mem::transmute(model.binding.jnt_type()[id.index()]) };
        if jnt_type == J::MJT {
            let slice = unsafe { data.binding.qpos(&model.binding) };
            let offset = model.jnt_qposadr(id);
            Ok(J::Qpos::try_from(&slice[offset..(offset + J::QPOS_SIZE)]).ok().unwrap())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }
    pub fn set_qpos<J: JointType>(&mut self, id: ObjectId<obj::Joint>, qpos: J::Qpos) -> Result<(), Error> {
        let Self { model, data } = self;
        let jnt_type = unsafe { std::mem::transmute(model.binding.jnt_type()[id.index()]) };
        if jnt_type == J::MJT {
            let slice = unsafe { data.binding.qpos_mut(&model.binding) };
            let offset = model.jnt_qposadr(id);
            slice[offset..(offset + J::QPOS_SIZE)].copy_from_slice(qpos.as_ref());
            Ok(())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }

    pub fn qvel<J: JointType>(&self, id: ObjectId<obj::Joint>) -> Result<J::Qvel, Error> {
        let Self { model, data } = self;
        let jnt_type = unsafe { std::mem::transmute(model.binding.jnt_type()[id.index()]) };
        if jnt_type == J::MJT {
            let slice = unsafe { data.binding.qvel(&model.binding) };
            let offset = model.jnt_dofadr(id).expect("Currently we don't support `weld` joint here, so this will not be None...");
            Ok(J::Qvel::try_from(&slice[offset..(offset + J::QVEL_SIZE)]).ok().unwrap())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }
    pub fn set_qvel<J: JointType>(&mut self, id: ObjectId<obj::Joint>, qvel: J::Qvel) -> Result<(), Error> {
        let Self { model, data } = self;
        let jnt_type = unsafe { std::mem::transmute(model.binding.jnt_type()[id.index()]) };
        if jnt_type == J::MJT {
            let slice = unsafe { data.binding.qvel_mut(&model.binding) };
            if let Some(offset) = model.jnt_dofadr(id) {
                slice[offset..(offset + J::QVEL_SIZE)].copy_from_slice(qvel.as_ref());
            }
            Ok(())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }

    pub fn qacc_warmstart(&self, id: ObjectId<obj::Dof>) -> f64 {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.qacc_warmstart(&model.binding) };
        slice[id.index()]
    }
    pub fn set_qacc_warmstart(&mut self, id: ObjectId<obj::Dof>, value: f64) {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.qacc_warmstart_mut(&model.binding) };
        slice[id.index()] = value;
    }

    pub fn plugin_state(&self, id: ObjectId<obj::Plugin>) -> Option<f64> {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.plugin_state(&model.binding) };
        let offset = model.plugin_stateadr(id)?;
        Some(slice[offset])
    }
    pub fn set_plugin_state(&mut self, id: ObjectId<obj::Plugin>, value: f64) -> Result<(), Error> {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.plugin_state_mut(&model.binding) };
        let offset = model.plugin_stateadr(id).ok_or(Error::PluginStateless(id))?;
        slice[offset] = value;
        Ok(())
    }

    pub fn qfrc_applied(&self, id: ObjectId<obj::Dof>) -> f64 {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.qfrc_applied(&model.binding) };
        slice[id.index()]
    }
    pub fn set_qfrc_applied(&mut self, id: ObjectId<obj::Dof>, value: f64) {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.qfrc_applied_mut(&model.binding) };
        slice[id.index()] = value;
    }

    pub fn xfrc_applied(&self, id: ObjectId<obj::Body>) -> [f64; 6] {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.xfrc_applied(&model.binding) };
        let offset = id.index() * 6;
        std::array::from_fn(|i| slice[offset + i])
    }
    pub fn set_xfrc_applied(&mut self, id: ObjectId<obj::Body>, value: [f64; 6]) {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.xfrc_applied_mut(&model.binding) };
        let offset = id.index() * 6;
        slice[offset..(offset + 6)].copy_from_slice(&value);
    }

    pub fn eq_active(&self, id: ObjectId<obj::Equality>) -> bool {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.eq_active(&model.binding) };
        slice[id.index()]
    }
    pub fn set_eq_active(&mut self, id: ObjectId<obj::Equality>, value: bool) {
        let Self { model, data } = self;
        let slice = unsafe { data.binding.eq_active_mut(&model.binding) };
        slice[id.index()] = value;
    }

    pub fn mocap_pos(&self, id: ObjectId<obj::Body>) -> Option<(f64, f64, f64)> {
        let Self { model, data } = self;
        let id = model.body_mocapid(id)?;
        let slice = unsafe { data.binding.mocap_pos(&model.binding) };
        let offset = id.index() * 3;
        Some((slice[offset], slice[offset + 1], slice[offset + 2]))
    }
    pub fn set_mocap_pos(&mut self, id: ObjectId<obj::Body>, pos: (f64, f64, f64)) -> Result<(), Error> {
        let Self { model, data } = self;
        let id = model.body_mocapid(id).ok_or(Error::BodyNotMocap(id))?;
        let slice = unsafe { data.binding.mocap_pos_mut(&model.binding) };
        let offset = id.index() * 3;
        slice[offset..(offset + 3)].copy_from_slice(&[pos.0, pos.1, pos.2]);
        Ok(())
    }

    pub fn mocap_quat(&self, id: ObjectId<obj::Body>) -> Option<(f64, f64, f64, f64)> {
        let Self { model, data } = self;
        let id = model.body_mocapid(id)?;
        let slice = unsafe { data.binding.mocap_quat(&model.binding) };
        let offset = id.index() * 4;
        Some((slice[offset], slice[offset + 1], slice[offset + 2], slice[offset + 3]))
    }
    pub fn set_mocap_quat(&mut self, id: ObjectId<obj::Body>, quat: (f64, f64, f64, f64)) -> Result<(), Error> {
        let Self { model, data } = self;
        let id = model.body_mocapid(id).ok_or(Error::BodyNotMocap(id))?;
        let slice = unsafe { data.binding.mocap_quat_mut(&model.binding) };
        let offset = id.index() * 4;
        slice[offset..(offset + 4)].copy_from_slice(&[quat.0, quat.1, quat.2, quat.3]);
        Ok(())
    }
    /**/
}
