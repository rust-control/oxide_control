pub use rusty_mujoco as binding;
pub use binding::{ObjectId, ObjType, Obj};

use crate::error::Error;

pub struct Model(binding::MjModel);

impl Model {
    pub(crate) fn new(model: binding::MjModel) -> Self {
        Self(model)
    }
}
impl std::ops::Deref for Model {
    type Target = binding::MjModel;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for Model {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

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

impl Data {
    pub(crate) fn new(data: binding::MjData) -> Self {
        Self(data)
    }
}
impl std::ops::Deref for Data {
    type Target = binding::MjData;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl std::ops::DerefMut for Data {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Data {}

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

pub struct Acturators<'a> {
    physics: &'a mut Physics,
    name_to_id: rustc_hash::FxHashMap<&'static str, ObjectId<binding::obj::Actuator>>,
}

impl Physics {
    pub fn actuators(&mut self) -> Acturators<'_> {
        Acturators {
            physics: self,
            name_to_id: rustc_hash::FxHashMap::default(),
        }
    }
}

impl<'a> Acturators<'a> {
    pub fn set(&mut self, name: &'static str, control: f64) -> Result<(), Error> {
        let id = match self.name_to_id.get(name) {
            Some(&id) => id,
            None => {
                let id = self.physics.model.object_id_of::<binding::obj::Actuator>(name)
                    .ok_or(Error::NameNotFound(name))?;
                self.name_to_id.insert(name, id);
                id
            }
        };

        let nu = self.physics.model().nu();
        (unsafe { self.physics.data_mut().ctrl_mut(nu) })[id.index()] = control;

        Ok(())
    }
}
