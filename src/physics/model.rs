use super::{Error, JointType};
use super::binding::{
    ObjectId, obj, helper::Rgba,
};
use super::binding::bindgen::{
    mjtSameFrame, mjtJoint, mjtGeom,
    mjNREF, mjNIMP, mjNFLUID,
};

pub struct Model { pub binding: super::binding::mjModel }

#[allow(non_snake_case)]
impl Model {
    /// qpos values at default pose
    pub fn qpos0<J: JointType>(&self, joint_id: ObjectId<obj::Joint>) -> Result<J::Qpos, Error> {
        let jnt_type = unsafe { std::mem::transmute(self.binding.jnt_type()[joint_id.index()]) };
        if jnt_type == J::MJT {
            let (it, offset) = (self.binding.qpos0(), self.binding.jnt_qposadr()[joint_id.index()] as usize);
            Ok(J::Qpos::try_from(&it[offset..(offset + J::QPOS_SIZE)]).ok().unwrap())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }
    /// reference pose for springs
    pub fn qpos_spring<J: JointType>(&self, joint_id: ObjectId<obj::Joint>) -> Result<J::Qpos, Error> {
        let jnt_type = unsafe { std::mem::transmute(self.binding.jnt_type()[joint_id.index()]) };
        if jnt_type == J::MJT {
            let (it, offset) = (self.binding.qpos_spring(), self.binding.jnt_qposadr()[joint_id.index()] as usize);
            Ok(J::Qpos::try_from(&it[offset..(offset + J::QPOS_SIZE)]).ok().unwrap())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }

    /// id of body's parent
    pub fn body_parentid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        let index = usize::try_from(self.binding.body_parentid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// id of root above body
    pub fn body_rootid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.body_rootid()[id.index()] as usize) }
    }
    /// id of body that this body is welded to
    pub fn body_weldid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.body_weldid()[id.index()] as usize) }
    }
    /// id of mocap data
    pub fn body_mocapid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        let index = usize::try_from(self.binding.body_parentid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// number of joints for this body
    pub fn body_jntnum(&self, id: ObjectId<obj::Body>) -> usize {
        self.binding.body_jntnum()[id.index()] as usize
    }
    /// start addr of joints
    pub fn body_jntadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        usize::try_from(self.binding.body_jntadr()[id.index()]).ok()
    }
    /// number of motion degrees of freedom
    pub fn body_dofnum(&self, id: ObjectId<obj::Body>) -> usize {
        self.binding.body_dofnum()[id.index()] as usize
    }
    /// start addr of dofs
    pub fn body_dofadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        usize::try_from(self.binding.body_dofadr()[id.index()]).ok()
    }
    /// id of body's kinematic tree
    pub fn body_treeid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        let index = usize::try_from(self.binding.body_parentid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// number of geoms
    pub fn body_geomnum(&self, id: ObjectId<obj::Body>) -> usize {
        self.binding.body_geomnum()[id.index()] as usize
    }
    /// start addr of geoms
    pub fn body_geomadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        usize::try_from(self.binding.body_geomadr()[id.index()]).ok()
    }
    /// body_simple: 1: diag M; 2: diag M, sliders only
    pub fn body_simple(&self, id: ObjectId<obj::Body>) -> u8 {
        self.binding.body_simple()[id.index()]
    }
    /// same frame as inertia
    pub fn body_sameframe(&self, id: ObjectId<obj::Body>) -> mjtSameFrame {
        unsafe { std::mem::transmute(self.binding.body_sameframe()[id.index()] as u32) }
    }
    /// position offset relative to parent body
    pub fn body_pos(&self, id: ObjectId<obj::Body>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.body_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// orientation offset relative to parent body
    pub fn body_quat(&self, id: ObjectId<obj::Body>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.body_quat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// local position of center of mass
    pub fn body_ipos(&self, id: ObjectId<obj::Body>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.body_ipos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// local orientation of inertia ellipsoid
    pub fn body_iquat(&self, id: ObjectId<obj::Body>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.body_iquat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// mass
    pub fn body_mass(&self, id: ObjectId<obj::Body>) -> f64 {
        self.binding.body_mass()[id.index()]
    }
    /// mass of subtree starting at this body
    pub fn body_subtreemass(&self, id: ObjectId<obj::Body>) -> f64 {
        self.binding.body_subtreemass()[id.index()]
    }
    /// diagonal inertia in ipos/iquat frame
    pub fn body_inertia(&self, id: ObjectId<obj::Body>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.body_inertia(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// mean inverse inertia in qpos0 (translation, rotation)
    pub fn body_invweight0(&self, id: ObjectId<obj::Body>) -> (f64, f64) {
        let (it, offset) = (self.binding.body_invweight0(), id.index() * 2);
        (it[offset], it[offset + 1])
    }
    /// antigravity force, units of body weight
    pub fn body_gravcomp(&self, id: ObjectId<obj::Body>) -> f64 {
        self.binding.body_gravcomp()[id.index()]
    }
    /// maximum over all geom margins
    pub fn body_margin(&self, id: ObjectId<obj::Body>) -> f64 {
        self.binding.body_margin()[id.index()]
    }
    /// plugin instance id
    pub fn body_plugin(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Plugin>> {
        let index = usize::try_from(self.binding.body_plugin()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// bit-wise OR over all geom contypes
    pub fn body_contype(&self, id: ObjectId<obj::Body>) -> i32 {
        self.binding.body_contype()[id.index()]
    }
    /// bit-wise OR over all geom conaffinities
    pub fn body_conaffinity(&self, id: ObjectId<obj::Body>) -> i32 {
        self.binding.body_conaffinity()[id.index()]
    }
    /// address of bounding volume hierarchy root
    pub fn body_bvhadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        usize::try_from(self.binding.body_bvhadr()[id.index()]).ok()
    }
    /// number of bounding volumes
    pub fn body_bvhnum(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        usize::try_from(self.binding.body_bvhnum()[id.index()]).ok()
    }

    /// depth in the bounding volume hierarchy
    pub fn bvh_depth(&self, id: ObjectId<obj::Geom>) -> usize {
        usize::try_from(self.binding.bvh_depth()[id.index()]).ok().unwrap()
    }
    /// left and right children in tree
    pub fn bvh_child(&self, id: ObjectId<obj::Geom>) -> (Option<usize>, Option<usize>) {
        let (it, offset) = (self.binding.bvh_child(), id.index() * 2);
        (usize::try_from(it[offset]).ok(), usize::try_from(it[offset + 1]).ok())
    }
    /// geom or elem id of node
    pub fn bvh_nodeid(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Geom>> {
        let index = usize::try_from(self.binding.bvh_nodeid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// local bounding box (center, size)
    pub fn bvh_aabb(&self, id: ObjectId<obj::Geom>) -> ((f64, f64, f64), (f64, f64, f64)) {
        let (it, offset) = (self.binding.bvh_aabb(), id.index() * 6);
        (
            (it[offset], it[offset + 1], it[offset + 2]),
            (it[offset + 3], it[offset + 4], it[offset + 5])
        )
    }

    /// type of joint
    pub fn jnt_type(&self, id: ObjectId<obj::Joint>) -> mjtJoint {
        unsafe { std::mem::transmute(self.binding.jnt_type()[id.index()] as u32) }
    }
    /// start addr in 'qpos' for joint's data
    pub fn jnt_qposadr(&self, id: ObjectId<obj::Joint>) -> usize {
        self.binding.jnt_qposadr()[id.index()] as usize
    }
    /// start addr in 'qvel' for joint's data
    pub fn jnt_dofadr(&self, id: ObjectId<obj::Joint>) -> Option<usize> {
        usize::try_from(self.binding.jnt_dofadr()[id.index()]).ok()
    }
    /// id of joint's body
    pub fn jnt_bodyid(&self, id: ObjectId<obj::Joint>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.jnt_bodyid()[id.index()] as usize) }
    }
    /// group for visibility
    pub fn jnt_group(&self, id: ObjectId<obj::Joint>) -> i32 {
        self.binding.jnt_group()[id.index()]
    }
    /// does joint have limits
    pub fn jnt_limited(&self, id: ObjectId<obj::Joint>) -> bool {
        self.binding.jnt_limited()[id.index()] != 0
    }
    /// does joint have actuator force limits
    pub fn jnt_actfrclimited(&self, id: ObjectId<obj::Joint>) -> bool {
        self.binding.jnt_actfrclimited()[id.index()] != 0
    }
    /// is gravcomp force applied via actuators
    pub fn jnt_actgravcomp(&self, id: ObjectId<obj::Joint>) -> bool {
        self.binding.jnt_actgravcomp()[id.index()] != 0
    }
    /// constraint solver reference: limit
    pub fn jnt_solref(&self, id: ObjectId<obj::Joint>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.jnt_solref(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance: limit
    pub fn jnt_solimp(&self, id: ObjectId<obj::Joint>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.jnt_solimp(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// local anchor position
    pub fn jnt_pos(&self, id: ObjectId<obj::Joint>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.jnt_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// local joint axis
    pub fn jnt_axis(&self, id: ObjectId<obj::Joint>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.jnt_axis(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// stiffness coefficient
    pub fn jnt_stiffness(&self, id: ObjectId<obj::Joint>) -> f64 {
        self.binding.jnt_stiffness()[id.index()]
    }
    /// joint limits
    pub fn jnt_range(&self, id: ObjectId<obj::Joint>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.jnt_range(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// range of total actuator force
    pub fn jnt_actfrcrange(&self, id: ObjectId<obj::Joint>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.jnt_actfrcrange(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// min distance for limit detection
    pub fn jnt_margin(&self, id: ObjectId<obj::Joint>) -> f64 {
        self.binding.jnt_margin()[id.index()]
    }

    /*
    dof_bodyid: [i32; nv * 1]           = "id of dof's body";
    dof_jntid: [i32; nv * 1]            = "id of dof's joint";
    dof_parentid: [i32; nv * 1]         = "id of dof's parent; -1: none";
    dof_treeid: [i32; nv * 1]           = "id of dof's kinematic tree";
    dof_Madr: [i32; nv * 1]             = "dof address in M-diagonal";
    dof_simplenum: [i32; nv * 1]        = "number of consecutive simple dofs";
    dof_solref: [f64; nv * mjNREF]      = "constraint solver reference:frictionloss";
    dof_solimp: [f64; nv * mjNIMP]      = "constraint solver impedance:frictionloss";
    dof_frictionloss: [f64; nv * 1]     = "dof friction loss";
    dof_armature: [f64; nv * 1]         = "dof armature inertia/mass";
    dof_damping: [f64; nv * 1]          = "damping coefficient";
    dof_invweight0: [f64; nv * 1]       = "diag. inverse inertia in qpos0";
    dof_M0: [f64; nv * 1]               = "diag. inertia in qpos0";
    */
    /// id of dof's body
    pub fn dof_bodyid(&self, id: ObjectId<obj::Dof>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.dof_bodyid()[id.index()] as usize) }
    }
    /// id of dof's joint
    pub fn dof_jntid(&self, id: ObjectId<obj::Dof>) -> ObjectId<obj::Joint> {
        unsafe { ObjectId::new_unchecked(self.binding.dof_jntid()[id.index()] as usize) }
    }
    /// id of dof's parent
    pub fn dof_parentid(&self, id: ObjectId<obj::Dof>) -> Option<ObjectId<obj::Dof>> {
        let index = usize::try_from(self.binding.dof_parentid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// id of dof's kinematic tree
    pub fn dof_treeid(&self, id: ObjectId<obj::Dof>) -> usize {
        self.binding.dof_treeid()[id.index()] as usize
    }
    /// dof address in M-diagonal
    pub fn dof_Madr(&self, id: ObjectId<obj::Dof>) -> usize {
        self.binding.dof_Madr()[id.index()] as usize
    }
    /// number of consecutive simple dofs
    pub fn dof_simplenum(&self, id: ObjectId<obj::Dof>) -> usize {
        self.binding.dof_simplenum()[id.index()] as usize
    }
    /// constraint solver reference: friction loss
    pub fn dof_solref(&self, id: ObjectId<obj::Dof>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.dof_solref(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance: friction loss
    pub fn dof_solimp(&self, id: ObjectId<obj::Dof>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.dof_solimp(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// dof friction loss
    pub fn dof_frictionloss(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.binding.dof_frictionloss()[id.index()]
    }
    /// dof armature inertia/mass
    pub fn dof_armature(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.binding.dof_armature()[id.index()]
    }
    /// damping coefficient
    pub fn dof_damping(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.binding.dof_damping()[id.index()]
    }
    /// diagonal inverse inertia in qpos0
    pub fn dof_invweight0(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.binding.dof_invweight0()[id.index()]
    }
    /// diagonal inertia in qpos0
    pub fn dof_M0(&self, id: ObjectId<obj::Dof>) -> f64 {
        self.binding.dof_M0()[id.index()]
    }

    /*
    geom_type: [i32; ngeom * 1]            = "geometric type (mjtGeom)";
    geom_contype: [i32; ngeom * 1]         = "geom contact type";
    geom_conaffinity: [i32; ngeom * 1]     = "geom contact affinity";
    geom_condim: [i32; ngeom * 1]          = "contact dimensionality (1, 3, 4, 6)";
    geom_bodyid: [i32; ngeom * 1]          = "id of geom's body";
    geom_dataid: [i32; ngeom * 1]          = "id of geom's mesh/hfield; -1: none";
    geom_matid: [i32; ngeom * 1]           = "material id for rendering; -1: none";
    geom_group: [i32; ngeom * 1]           = "group for visibility";
    geom_priority: [i32; ngeom * 1]        = "geom contact priority";
    geom_plugin: [i32; ngeom * 1]          = "plugin instance id; -1: not in use";
    geom_sameframe: [u8; ngeom * 1]        = "same frame as body (mjtSameframe)";
    geom_solmix: [f64; ngeom * 1]          = "mixing coef for solref/imp in geom pair";
    geom_solref: [f64; ngeom * mjNREF]     = "constraint solver reference: contact";
    geom_solimp: [f64; ngeom * mjNIMP]     = "constraint solver impedance: contact";
    geom_size: [f64; ngeom * 3]            = "geom-specific size parameters";
    geom_aabb: [f64; ngeom * 6]            = "bounding box, (center, size)";
    geom_rbound: [f64; ngeom * 1]          = "radius of bounding sphere";
    geom_pos: [f64; ngeom * 3]             = "local position offset rel. to body";
    geom_quat: [f64; ngeom * 4]            = "local orientation offset rel. to body";
    geom_friction: [f64; ngeom * 3]        = "friction for (slide, spin, roll)";
    geom_margin: [f64; ngeom * 1]          = "detect contact if dist<margin";
    geom_gap: [f64; ngeom * 1]             = "include in solver if dist<margin-gap";
    geom_fluid: [f64; ngeom * mjNFLUID]    = "fluid interaction parameters";
    geom_rgba: [f32; ngeom * 4]            = "rgba when material is omitted";
    */
    /// geometric type (mjtGeom)
    pub fn geom_type(&self, id: ObjectId<obj::Geom>) -> mjtGeom {
        unsafe { std::mem::transmute(self.binding.geom_type()[id.index()] as u32) }
    }
    /// geom contact type
    pub fn geom_contype(&self, id: ObjectId<obj::Geom>) -> i32 {
        self.binding.geom_contype()[id.index()]
    }
    /// geom contact affinity
    pub fn geom_conaffinity(&self, id: ObjectId<obj::Geom>) -> i32 {
        self.binding.geom_conaffinity()[id.index()]
    }
    /// contact dimensionality (1, 3, 4, 6)
    pub fn geom_condim(&self, id: ObjectId<obj::Geom>) -> usize {
        self.binding.geom_condim()[id.index()] as usize
    }
    /// id of geom's body
    pub fn geom_bodyid(&self, id: ObjectId<obj::Geom>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.geom_bodyid()[id.index()] as usize) }
    }
    /// id of geom's mesh
    pub fn geom_dataid_mesh(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Mesh>> {
        if self.geom_type(id) != mjtGeom::MESH {return None}
        let index = usize::try_from(self.binding.geom_dataid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// id of geom's hfield
    pub fn geom_dataid_hfield(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::HField>> {
        if self.geom_type(id) != mjtGeom::HFIELD {return None}
        let index = usize::try_from(self.binding.geom_dataid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// material id for rendering
    pub fn geom_matid(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Material>> {
        let index = usize::try_from(self.binding.geom_matid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn geom_group(&self, id: ObjectId<obj::Geom>) -> i32 {
        self.binding.geom_group()[id.index()]
    }
    /// geom contact priority
    pub fn geom_priority(&self, id: ObjectId<obj::Geom>) -> i32 {
        self.binding.geom_priority()[id.index()]
    }
    /// plugin instance id
    pub fn geom_plugin(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Plugin>> {
        let index = usize::try_from(self.binding.geom_plugin()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// same frame as body (mjtSameframe)
    pub fn geom_sameframe(&self, id: ObjectId<obj::Geom>) -> mjtSameFrame {
        unsafe { std::mem::transmute(self.binding.geom_sameframe()[id.index()] as u32) }
    }
    /// mixing coef for solref/imp in geom pair
    pub fn geom_solmix(&self, id: ObjectId<obj::Geom>) -> f64 {
        self.binding.geom_solmix()[id.index()]
    }
    /// constraint solver reference: contact
    pub fn geom_solref(&self, id: ObjectId<obj::Geom>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.geom_solref(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance: contact
    pub fn geom_solimp(&self, id: ObjectId<obj::Geom>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.geom_solimp(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// geom-specific size parameters
    pub fn geom_size(&self, id: ObjectId<obj::Geom>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.geom_size(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// bounding box, (center, size)
    pub fn geom_aabb(&self, id: ObjectId<obj::Geom>) -> ((f64, f64, f64), (f64, f64, f64)) {
        let (it, offset) = (self.binding.geom_aabb(), id.index() * 6);
        (
            (it[offset], it[offset + 1], it[offset + 2]),
            (it[offset + 3], it[offset + 4], it[offset + 5])
        )
    }
    /// radius of bounding sphere
    pub fn geom_rbound(&self, id: ObjectId<obj::Geom>) -> f64 {
        self.binding.geom_rbound()[id.index()]
    }
    /// local position offset relative to body
    pub fn geom_pos(&self, id: ObjectId<obj::Geom>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.geom_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// local orientation offset relative to body
    pub fn geom_quat(&self, id: ObjectId<obj::Geom>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.geom_quat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// friction for (slide, spin, roll)
    pub fn geom_friction(&self, id: ObjectId<obj::Geom>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.geom_friction(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// detect contact if dist < margin
    pub fn geom_margin(&self, id: ObjectId<obj::Geom>) -> f64 {
        self.binding.geom_margin()[id.index()]
    }
    /// include in solver if dist < margin - gap
    pub fn geom_gap(&self, id: ObjectId<obj::Geom>) -> f64 {
        self.binding.geom_gap()[id.index()]
    }
    /// fluid interaction parameters
    pub fn geom_fluid(&self, id: ObjectId<obj::Geom>) -> [f64; mjNFLUID] {
        let (it, offset) = (self.binding.geom_fluid(), id.index() * mjNFLUID);
        [
            it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4], it[offset + 5],
            it[offset + 6], it[offset + 7], it[offset + 8], it[offset + 9], it[offset + 10], it[offset + 11]
        ]
    }
    /// rgba when material is omitted
    pub fn geom_rgba(&self, id: ObjectId<obj::Geom>) -> Rgba {
        let (it, offset) = (self.binding.geom_rgba(), id.index() * 4);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: it[offset + 3] }
    }

    /*    
    site_type: [i32; nsite * 1]            = "geom type for rendering (mjtGeom)";
    site_bodyid: [i32; nsite * 1]          = "id of site's body";
    site_matid: [i32; nsite * 1]           = "material id for rendering; -1: none";
    site_group: [i32; nsite * 1]           = "group for visibility";
    site_sameframe: [u8; nsite * 1]        = "same frame as body (mjtSameframe)";
    site_size: [f64; nsite * 3]            = "geom size for rendering";
    site_pos: [f64; nsite * 3]             = "local position offset rel. to body";
    site_quat: [f64; nsite * 4]            = "local orientation offset rel. to body";
    site_user: [f64; nsite x nuser_site]   = "user data";
    site_rgba: [f32; nsite * 4]            = "rgba when material is omitted";
    */
/* */
}
