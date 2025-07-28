pub use rusty_mujoco as binding;
pub use binding::{mjModel, mjData, ObjectId, obj, Joint, joint, mjMAXVAL, mjMINVAL};

use crate::error::Error;

pub struct Physics {
    model: mjModel,
    data: mjData,
}

impl Physics {
    pub fn from_xml(xml_path: impl AsRef<std::path::Path>) -> Result<Self, Error> {
        let model = binding::mj_loadXML(xml_path.as_ref().to_str().unwrap())?;
        let data = binding::mj_makeData(&model);
        Ok(Self { model, data })
    }

    pub fn from_xml_string(xml_string: impl Into<String>) -> Result<Self, Error> {
        let mut spec = binding::mj_parseXMLString(xml_string.into())?;
        let model = binding::mj_compile(&mut spec)
            .ok_or_else(|| Error::Mjs(binding::mjs_getError(&mut spec).unwrap_or_else(String::new)))?;
        let data = binding::mj_makeData(&model);
        Ok(Self { model, data })
    }

    pub fn model(&self) -> &mjModel {
        &self.model
    }

    pub fn data(&self) -> &mjData {
        &self.data
    }
    pub fn data_mut(&mut self) -> &mut mjData {
        &mut self.data
    }

    pub fn model_data(&self) -> (&mjModel, &mjData) {
        (&self.model, &self.data)
    }
    pub fn model_datamut(&mut self) -> (&mjModel, &mut mjData) {
        (&self.model, &mut self.data)
    }

    pub fn step(&mut self) {
        rusty_mujoco::mj_step(&self.model, &mut self.data);
    }

    pub fn forward(&mut self) {
        rusty_mujoco::mj_forward(&self.model, &mut self.data);
    }

    pub fn reset(&mut self) {
        rusty_mujoco::mj_resetData(&self.model, &mut self.data);
    }

    pub fn object_id<O: binding::Obj>(&self, name: &str) -> Option<ObjectId<O>> {
        self.model.object_id(name)
    }

    pub fn object_name<O: binding::Obj>(&self, id: ObjectId<O>) -> String {
        binding::mj_id2name::<O>(&self.model, id)
    }
}

pub struct Actuators<'a> {
    physics: &'a mut Physics,
}
impl<'a> Actuators<'a> {
    pub fn set(&mut self, id: ObjectId<obj::Actuator>, control: f64) {
        self.physics.set_ctrl(id, control);
    }
}
impl Physics {
    pub fn actuators(&mut self) -> Actuators<'_> {
        Actuators {
            physics: self,
        }
    }
}

impl Physics {
    pub fn time(&self) -> f64 {
        self.data.time()
    }
    pub fn set_time(&mut self, time: f64) {
        self.data.set_time(time);
    }

    pub fn ctrl(&self, id: ObjectId<obj::Actuator>) -> f64 {
        self.data.ctrl(id)
    }
    pub fn set_ctrl(&mut self, id: ObjectId<obj::Actuator>, value: f64) {
        self.data.set_ctrl(id, value);
    }

    pub fn act(&self, id: ObjectId<obj::Actuator>) -> Option<f64> {
        self.data.act(id, &self.model)
    }
    /// Set the actuator activation value. `None` when the actuator is stateless.
    pub fn set_act(&mut self, id: ObjectId<obj::Actuator>, value: f64) -> Option<()> {
        self.data.set_act(id, value, &self.model)
    }

    pub fn qpos<J: Joint>(&self, id: ObjectId<J>) -> J::Qpos {
        self.data.qpos(id, &self.model)
    }
    pub fn set_qpos<J: Joint>(&mut self, id: ObjectId<J>, qpos: J::Qpos) {
        self.data.set_qpos(id, qpos, &self.model);
    }

    pub fn qvel<J: Joint>(&self, id: ObjectId<J>) -> J::Qvel {
        self.data.qvel(id, &self.model)
    }
    pub fn set_qvel<J: Joint>(&mut self, id: ObjectId<J>, qvel: J::Qvel) {
        self.data.set_qvel(id, qvel, &self.model);
    }

    pub fn qacc_warmstart(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.data.qacc_warmstart(id)
    }
    pub fn set_qacc_warmstart(&mut self, id: ObjectId<obj::Dof>, value: f64) {
        self.data.set_qacc_warmstart(id, value);
    }

    pub fn plugin_state(&self, id: ObjectId<obj::Plugin>) -> Option<f64> {
        self.data.plugin_state(id, &self.model)
    }
    /// Set the plugin state. Returns `None` if the plugin does not have a state.
    pub fn set_plugin_state(&mut self, id: ObjectId<obj::Plugin>, value: f64) -> Option<()> {
        self.data.set_plugin_state(id, value, &self.model)
    }

    pub fn qfrc_applied(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.data.qfrc_applied(id)
    }
    pub fn set_qfrc_applied(&mut self, id: ObjectId<obj::Dof>, value: f64) {
        self.data.set_qfrc_applied(id, value);
    }

    pub fn xfrc_applied(&self, id: ObjectId<obj::Body>) -> [f64; 6] {
        self.data.xfrc_applied(id)
    }
    pub fn set_xfrc_applied(&mut self, id: ObjectId<obj::Body>, value: [f64; 6]) {
        self.data.set_xfrc_applied(id, value);
    }

    pub fn eq_active(&self, id: ObjectId<obj::Equality>) -> bool {
        self.data.eq_active(id)
    }
    pub fn set_eq_active(&mut self, id: ObjectId<obj::Equality>, value: bool) {
        self.data.set_eq_active(id, value);
    }

    /// `None` when the body is not a mocap body.
    pub fn mocap_pos(&self, id: ObjectId<obj::Body>) -> Option<[f64; 3]> {
        self.data.mocap_pos(id, &self.model)
    }
    /// Set the mocap position. Returns `None` if the body is not a mocap body.
    pub fn set_mocap_pos(&mut self, id: ObjectId<obj::Body>, pos: [f64; 3]) -> Option<()> {
        self.data.set_mocap_pos(id, pos, &self.model)
    }

    /// `None` when the body is not a mocap body.
    pub fn mocap_quat(&self, id: ObjectId<obj::Body>) -> Option<[f64; 4]> {
        self.data.mocap_quat(id, &self.model)
    }
    /// Set the mocap quaternion. Returns `None` if the body is not a mocap body.
    pub fn set_mocap_quat(&mut self, id: ObjectId<obj::Body>, quat: [f64; 4]) -> Option<()> {
        self.data.set_mocap_quat(id, quat, &self.model)
    }
}
