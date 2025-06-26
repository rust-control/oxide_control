use crate::error::Error;
use crate::mujoco::{Model, Data, ObjectId, Obj, binding};

pub struct Physics {
    model: Model,
    data: Data,
}

impl Physics {
    pub fn from_xml(xml_path: impl AsRef<std::path::Path>) -> Result<Self, Error> {
        let model = binding::mj_loadXML(xml_path.as_ref().to_str().unwrap())?;
        let data = binding::mj_makeData(&model);
        Ok(Self {
            model: Model::new(model),
            data: Data::new(data),
        })
    }

    pub fn from_xml_string(xml_string: impl Into<String>) -> Result<Self, Error> {
        let mut spec = binding::mj_parseXMLString(xml_string.into())?;
        let model = binding::mj_compile(&mut spec)
            .ok_or_else(|| Error::Mjs(binding::mjs_getError(&mut spec).unwrap_or_else(String::new)))?;
        let data = binding::mj_makeData(&model);
        Ok(Self {
            model: Model::new(model),
            data: Data::new(data),
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
}

impl Physics {
    pub fn step(&mut self) {
        rusty_mujoco::mj_step(&self.model, &mut self.data);
    }

    pub fn forward(&mut self) {
        rusty_mujoco::mj_forward(&self.model, &mut self.data);
    }

    pub fn reset(&mut self) {
        rusty_mujoco::mj_resetData(&self.model, &mut self.data);
    }

    pub fn time(&self) -> f64 {
        self.data.time()
    }

    pub fn control(&mut self) -> &mut [f64] {
        unsafe { self.data.ctrl_mut(self.model.nu()) }
    }
}
