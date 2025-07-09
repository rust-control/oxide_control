use super::{Error, JointType};
use super::binding::{
    obj, Obj, ObjectId,
    ElementDataId, EdgeDataId, ShellDataId, NodeId, VertexId, EdgeId, ElementId, EvPairId, TexcoordId, NormalId, FaceId, BoneId, BoneVertexId, HFieldDataId, TexDataId,
    helper::Rgba,
};
use super::binding::bindgen::{
    mjtSameFrame, mjtJoint, mjtGeom, mjtCamLight, mjtFlexSelf, mjtTexture, mjtEq, mjtObj, mjtWrap, mjtTrn, mjtDyn, mjtGain, mjtBias, mjtSensor, mjtDataType, mjtStage,
    mjNREF, mjNIMP, mjNFLUID, mjNTEXROLE, mjNEQDATA, mjNDYN, mjNGAIN, mjNBIAS,
};
use std::mem::transmute as tx;

pub struct Model { pub binding: super::binding::mjModel }

#[allow(non_snake_case)]
impl Model {
    /// qpos values at default pose
    pub fn qpos0<J: JointType>(&self, joint_id: ObjectId<obj::Joint>) -> Result<J::Qpos, Error> {
        let jnt_type = unsafe { tx(self.binding.jnt_type()[joint_id.index()]) };
        if jnt_type == J::MJT {
            let (it, offset) = (self.binding.qpos0(), self.binding.jnt_qposadr()[joint_id.index()] as usize);
            Ok(J::Qpos::try_from(&it[offset..(offset + J::QPOS_SIZE)]).ok().unwrap())
        } else {
            Err(Error::JointTypeNotMatch { expected: J::MJT, found: jnt_type })
        }
    }
    /// reference pose for springs
    pub fn qpos_spring<J: JointType>(&self, joint_id: ObjectId<obj::Joint>) -> Result<J::Qpos, Error> {
        let jnt_type = unsafe { tx(self.binding.jnt_type()[joint_id.index()]) };
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
        unsafe { tx(self.binding.body_sameframe()[id.index()] as u32) }
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
        unsafe { tx(self.binding.jnt_type()[id.index()] as u32) }
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

    /// geometric type (mjtGeom)
    pub fn geom_type(&self, id: ObjectId<obj::Geom>) -> mjtGeom {
        unsafe { tx(self.binding.geom_type()[id.index()] as u32) }
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
        unsafe { tx(self.binding.geom_sameframe()[id.index()] as u32) }
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

    /// geom type for rendering
    pub fn site_type(&self, id: ObjectId<obj::Site>) -> mjtGeom {
        unsafe { tx(self.binding.site_type()[id.index()] as u32) }
    }
    /// id of site's body
    pub fn site_bodyid(&self, id: ObjectId<obj::Site>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.site_bodyid()[id.index()] as usize) }
    }
    /// material id for rendering
    pub fn site_matid(&self, id: ObjectId<obj::Site>) -> Option<ObjectId<obj::Material>> {
        let index = usize::try_from(self.binding.site_matid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn site_group(&self, id: ObjectId<obj::Site>) -> i32 {
        self.binding.site_group()[id.index()]
    }
    /// same frame as body
    pub fn site_sameframe(&self, id: ObjectId<obj::Site>) -> mjtSameFrame {
        unsafe { tx(self.binding.site_sameframe()[id.index()] as u32) }
    }
    /// geom size for rendering
    pub fn site_size(&self, id: ObjectId<obj::Site>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.site_size(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// local position offset rel. to body
    pub fn site_pos(&self, id: ObjectId<obj::Site>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.site_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// local orientation offset rel. to body
    pub fn site_quat(&self, id: ObjectId<obj::Site>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.site_quat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// rgba when material is omitted
    pub fn site_rgba(&self, id: ObjectId<obj::Site>) -> Rgba {
        let (it, offset) = (self.binding.site_rgba(), id.index() * 4);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: it[offset + 3] }
    }

    /// camera tracking mode (mjtCamLight)
    pub fn cam_mode(&self, id: ObjectId<obj::Camera>) -> mjtCamLight {
        unsafe { tx(self.binding.cam_mode()[id.index()] as u32) }
    }
    /// id of camera's body
    pub fn cam_bodyid(&self, id: ObjectId<obj::Camera>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.cam_bodyid()[id.index()] as usize) }
    }
    /// id of targeted body
    pub fn cam_targetbodyid(&self, id: ObjectId<obj::Camera>) -> Option<ObjectId<obj::Body>> {
        let index = usize::try_from(self.binding.cam_targetbodyid()[id.index()]).ok()?;
        unsafe { Some(ObjectId::new_unchecked(index)) }
    }
    /// does camera cast shadows
    pub fn cam_pos(&self, id: ObjectId<obj::Camera>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.cam_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// orientation relative to body frame
    pub fn cam_quat(&self, id: ObjectId<obj::Camera>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.cam_quat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// global position relative to sub-com in qpos0
    pub fn cam_poscom0(&self, id: ObjectId<obj::Camera>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.cam_poscom0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// global position relative to body in qpos0
    pub fn cam_pos0(&self, id: ObjectId<obj::Camera>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.cam_pos0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// global orientation in qpos0
    pub fn cam_mat0(&self, id: ObjectId<obj::Camera>) -> [[f64; 3]; 3] {
        let (it, offset) = (self.binding.cam_mat0(), id.index() * 9);
        [
            [it[offset], it[offset + 1], it[offset + 2]],
            [it[offset + 3], it[offset + 4], it[offset + 5]],
            [it[offset + 6], it[offset + 7], it[offset + 8]]
        ]
    }
    /// orthographic camera
    pub fn cam_orthographic(&self, id: ObjectId<obj::Camera>) -> bool {
        self.binding.cam_orthographic()[id.index()] != 0
    }
    /// y field-of-view (ortho ? len : deg)
    pub fn cam_fovy(&self, id: ObjectId<obj::Camera>) -> f64 {
        self.binding.cam_fovy()[id.index()]
    }
    /// inter-pupilary distance
    pub fn cam_ipd(&self, id: ObjectId<obj::Camera>) -> f64 {
        self.binding.cam_ipd()[id.index()]
    }
    /// resolution: pixels (width, height)
    pub fn cam_resolution(&self, id: ObjectId<obj::Camera>) -> (usize, usize) {
        let (it, offset) = (self.binding.cam_resolution(), id.index() * 2);
        (usize::try_from(it[offset]).ok().unwrap(), usize::try_from(it[offset + 1]).ok().unwrap())
    }
    /// sensor size: length (width, height)
    pub fn cam_sensorsize(&self, id: ObjectId<obj::Camera>) -> (f32, f32) {
        let (it, offset) = (self.binding.cam_sensorsize(), id.index() * 2);
        (it[offset], it[offset + 1])
    }
    /// [focal length; principal point]
    pub fn cam_intrinsic(&self, id: ObjectId<obj::Camera>) -> [f32; 4] {
        let (it, offset) = (self.binding.cam_intrinsic(), id.index() * 4);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3]]
    }

    /// light tracking mode (mjtCamLight)
    pub fn light_mode(&self, id: ObjectId<obj::Light>) -> mjtCamLight {
        unsafe { tx(self.binding.light_mode()[id.index()] as u32) }
    }
    /// id of light's body
    pub fn light_bodyid(&self, id: ObjectId<obj::Light>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.light_bodyid()[id.index()] as usize) }
    }
    /// id of targeted body
    pub fn light_targetbodyid(&self, id: ObjectId<obj::Light>) -> Option<ObjectId<obj::Body>> {
        let index = usize::try_from(self.binding.light_targetbodyid()[id.index()]).ok()?;
        unsafe { Some(ObjectId::new_unchecked(index)) }
    }
    /// does light cast shadows
    pub fn light_castshadow(&self, id: ObjectId<obj::Light>) -> bool {
        self.binding.light_castshadow()[id.index()] != 0
    }
    /// light radius for soft shadows
    pub fn light_bulbradius(&self, id: ObjectId<obj::Light>) -> f32 {
        self.binding.light_bulbradius()[id.index()]
    }
    /// is light on
    pub fn light_active(&self, id: ObjectId<obj::Light>) -> bool {
        self.binding.light_active()[id.index()] != 0
    }
    /// position relative to body frame
    pub fn light_pos(&self, id: ObjectId<obj::Light>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.light_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// direction relative to body frame
    pub fn light_dir(&self, id: ObjectId<obj::Light>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.light_dir(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// global position relative to sub-com in qpos0
    pub fn light_poscom0(&self, id: ObjectId<obj::Light>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.light_poscom0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// global position relative to body in qpos0
    pub fn light_pos0(&self, id: ObjectId<obj::Light>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.light_pos0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// global direction in qpos0
    pub fn light_dir0(&self, id: ObjectId<obj::Light>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.light_dir0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// OpenGL attenuation (quadratic model)
    pub fn light_attenuation(&self, id: ObjectId<obj::Light>) -> [f32; 3] {
        let (it, offset) = (self.binding.light_attenuation(), id.index() * 3);
        [it[offset], it[offset + 1], it[offset + 2]]
    }
    /// OpenGL cutoff
    pub fn light_cutoff(&self, id: ObjectId<obj::Light>) -> f32 {
        self.binding.light_cutoff()[id.index()]
    }
    /// OpenGL exponent
    pub fn light_exponent(&self, id: ObjectId<obj::Light>) -> f32 {
        self.binding.light_exponent()[id.index()]
    }
    /// ambient rgb (alpha=1)
    pub fn light_ambient(&self, id: ObjectId<obj::Light>) -> Rgba {
        let (it, offset) = (self.binding.light_ambient(), id.index() * 3);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: 1.0 }
    }
    /// diffuse rgb (alpha=1)
    pub fn light_diffuse(&self, id: ObjectId<obj::Light>) -> Rgba {
        let (it, offset) = (self.binding.light_diffuse(), id.index() * 3);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: 1.0 }
    }
    /// specular rgb (alpha=1)
    pub fn light_specular(&self, id: ObjectId<obj::Light>) -> Rgba {
        let (it, offset) = (self.binding.light_specular(), id.index() * 3);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: 1.0 }
    }

    /// flex contact type
    pub fn flex_contype(&self, id: ObjectId<obj::Flex>) -> i32 {
        self.binding.flex_contype()[id.index()]
    }
    /// flex contact affinity
    pub fn flex_conaffinity(&self, id: ObjectId<obj::Flex>) -> i32 {
        self.binding.flex_conaffinity()[id.index()]
    }
    /// contact dimensionality (1, 3, 4, 6)
    pub fn flex_condim(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_condim()[id.index()] as usize
    }
    /// flex contact priority
    pub fn flex_priority(&self, id: ObjectId<obj::Flex>) -> i32 {
        self.binding.flex_priority()[id.index()]
    }
    /// mix coef for solref/imp in contact pair
    pub fn flex_solmix(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_solmix()[id.index()]
    }
    /// constraint solver reference: contact
    pub fn flex_solref(&self, id: ObjectId<obj::Flex>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.flex_solref(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance: contact
    pub fn flex_solimp(&self, id: ObjectId<obj::Flex>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.flex_solimp(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// friction for (slide, spin, roll)
    pub fn flex_friction(&self, id: ObjectId<obj::Flex>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.flex_friction(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// detect contact if dist < margin
    pub fn flex_margin(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_margin()[id.index()]
    }
    /// include in solver if dist < margin - gap
    pub fn flex_gap(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_gap()[id.index()]
    }
    /// internal flex collision enabled
    pub fn flex_internal(&self, id: ObjectId<obj::Flex>) -> bool {
        self.binding.flex_internal()[id.index()] != 0
    }
    /// self collision mode (mjtFlexSelf)
    pub fn flex_selfcollide(&self, id: ObjectId<obj::Flex>) -> mjtFlexSelf {
        unsafe { tx(self.binding.flex_selfcollide()[id.index()] as u32) }
    }
    /// number of active element layers, 3D only
    pub fn flex_activelayers(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_activelayers()[id.index()] as usize
    }

    /// 1: lines, 2: triangles, 3: tetrahedra
    pub fn flex_dim(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_dim()[id.index()] as usize
    }
    /// material id for rendering
    pub fn flex_matid(&self, id: ObjectId<obj::Flex>) -> Option<ObjectId<obj::Material>> {
        let index = usize::try_from(self.binding.flex_matid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn flex_group(&self, id: ObjectId<obj::Flex>) -> i32 {
        self.binding.flex_group()[id.index()]
    }
    /// interpolation (0: vertex, 1: nodes)
    pub fn flex_interp(&self, id: ObjectId<obj::Flex>) -> i32 {
        self.binding.flex_interp()[id.index()]
    }
    /// first node address
    pub fn flex_nodeadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_nodeadr()[id.index()]).ok()
    }
    /// number of nodes
    pub fn flex_nodenum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_nodenum()[id.index()] as usize
    }
    /// first vertex address
    pub fn flex_vertadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_vertadr()[id.index()]).ok()
    }
    /// number of vertices
    pub fn flex_vertnum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_vertnum()[id.index()] as usize
    }
    /// first edge address
    pub fn flex_edgeadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_edgeadr()[id.index()]).ok()
    }
    /// number of edges
    pub fn flex_edgenum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_edgenum()[id.index()] as usize
    }
    /// first element address
    pub fn flex_elemadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_elemadr()[id.index()]).ok()
    }
    /// number of elements
    pub fn flex_elemnum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_elemnum()[id.index()] as usize
    }
    /// first element vertex id address
    pub fn flex_elemdataadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_elemdataadr()[id.index()]).ok()
    }
    /// first element edge id address
    pub fn flex_elemedgeadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_elemedgeadr()[id.index()]).ok()
    }
    /// number of shells
    pub fn flex_shellnum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_shellnum()[id.index()] as usize
    }
    /// first shell data address
    pub fn flex_shelldataadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_shelldataadr()[id.index()]).ok()
    }
    /// first evpair address
    pub fn flex_evpairadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_evpairadr()[id.index()]).ok()
    }
    /// number of evpairs
    pub fn flex_evpairnum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_evpairnum()[id.index()] as usize
    }
    /// node body ids
    pub fn flex_nodebodyid(&self, id: NodeId<obj::Flex>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.flex_nodebodyid()[id.index()] as usize) }
    }
    /// vertex body ids
    pub fn flex_vertbodyid(&self, id: VertexId<obj::Flex>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.flex_vertbodyid()[id.index()] as usize) }
    }
    /// edge vertex ids (2 per edge)
    pub fn flex_edge(&self, id: EdgeId<obj::Flex>) -> (VertexId<obj::Flex>, VertexId<obj::Flex>) {
        let (it, offset) = (self.binding.flex_edge(), id.index() * 2);
        (
            unsafe { VertexId::new_unchecked(it[offset] as usize) },
            unsafe { VertexId::new_unchecked(it[offset + 1] as usize) }
        )
    }
    /// element vertex ids (dim+1 per elem)
    pub fn flex_elem(&self, id: ElementDataId) -> VertexId<obj::Flex> {
        unsafe { VertexId::new_unchecked(self.binding.flex_elem()[id.index()] as usize) }
    }
    /// element texture coordinates (dim+1)
    pub fn flex_elemtexcoord(&self, id: ElementDataId) -> usize {
        self.binding.flex_elemtexcoord()[id.index()] as usize
    }
    /// element edge ids
    pub fn flex_elemedge(&self, id: EdgeDataId) -> EdgeId<obj::Flex> {
        unsafe { EdgeId::new_unchecked(self.binding.flex_elemedge()[id.index()] as usize) }
    }
    /// element distance from surface, 3D only
    pub fn flex_elemlayer(&self, id: ElementId<obj::Flex>) -> i32 {
        self.binding.flex_elemlayer()[id.index()]
    }
    /// shell fragment vertex ids (dim per frag)
    pub fn flex_shell(&self, id: ShellDataId) -> VertexId<obj::Flex> {
        unsafe { VertexId::new_unchecked(self.binding.flex_shell()[id.index()] as usize) }
    }
    /// (element, vertex) collision pairs
    pub fn flex_evpair(&self, id: EvPairId) -> (ElementId<obj::Flex>, VertexId<obj::Flex>) {
        let (it, offset) = (self.binding.flex_evpair(), id.index() * 2);
        (
            unsafe { ElementId::new_unchecked(it[offset] as usize) },
            unsafe { VertexId::new_unchecked(it[offset + 1] as usize) }
        )
    }
    /// vertex positions in local body frames
    pub fn flex_vert(&self, id: VertexId<obj::Flex>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.flex_vert(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// vertex positions in qpos0 on [0, 1]^d
    pub fn flex_vert0(&self, id: VertexId<obj::Flex>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.flex_vert0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// node positions in local body frames
    pub fn flex_node(&self, id: NodeId<obj::Flex>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.flex_node(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// Cartesian node positions in qpos0
    pub fn flex_node0(&self, id: NodeId<obj::Flex>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.flex_node0(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// edge lengths in qpos0
    pub fn flexedge_length0(&self, id: EdgeId<obj::Flex>) -> f64 {
        self.binding.flexedge_length0()[id.index()]
    }
    /// edge inv. weight in qpos0
    pub fn flexedge_invweight0(&self, id: EdgeId<obj::Flex>) -> f64 {
        self.binding.flexedge_invweight0()[id.index()]
    }
    /// radius around primitive element
    pub fn flex_radius(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_radius()[id.index()]
    }
    /// finite element stiffness matrix
    pub fn flex_stiffness(&self, id: ElementId<obj::Flex>) -> [f64; 21] {
        let (it, offset) = (self.binding.flex_stiffness(), id.index() * 21);
        std::array::from_fn(|i| it[offset + i])
    }
    /// Rayleigh's damping coefficient
    pub fn flex_damping(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_damping()[id.index()]
    }
    /// edge stiffness
    pub fn flex_edgestiffness(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_edgestiffness()[id.index()]
    }
    /// edge damping
    pub fn flex_edgedamping(&self, id: ObjectId<obj::Flex>) -> f64 {
        self.binding.flex_edgedamping()[id.index()]
    }
    /// is edge equality constraint defined
    pub fn flex_edgeequality(&self, id: EdgeId<obj::Flex>) -> bool {
        self.binding.flex_edgeequality()[id.index()] != 0
    }
    /// are all verices in the same body
    pub fn flex_rigid(&self, id: ObjectId<obj::Flex>) -> bool {
        self.binding.flex_rigid()[id.index()] != 0
    }
    /// are both edge vertices in same body
    pub fn flexedge_rigid(&self, id: EdgeId<obj::Flex>) -> bool {
        self.binding.flexedge_rigid()[id.index()] != 0
    }
    /// are all vertex coordinates (0,0,0)
    pub fn flex_centered(&self, id: ObjectId<obj::Flex>) -> bool {
        self.binding.flex_centered()[id.index()] != 0
    }
    /// render flex skin with flat shading
    pub fn flex_flatskin(&self, id: ObjectId<obj::Flex>) -> bool {
        self.binding.flex_flatskin()[id.index()] != 0
    }
    /// address of bvh root
    pub fn flex_bvhadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        usize::try_from(self.binding.flex_bvhadr()[id.index()]).ok()
    }
    /// number of bounding volumes
    pub fn flex_bvhnum(&self, id: ObjectId<obj::Flex>) -> usize {
        self.binding.flex_bvhnum()[id.index()] as usize
    }
    /// rgba when material is omitted
    pub fn flex_rgba(&self, id: ObjectId<obj::Flex>) -> Rgba {
        let (it, offset) = (self.binding.flex_rgba(), id.index() * 4);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: it[offset + 3] }
    }
    /// vertex texture coordinates
    pub fn flex_texcoord(&self, id: TexcoordId<obj::Flex>) -> (f32, f32) {
        let (it, offset) = (self.binding.flex_texcoord(), id.index() * 2);
        (it[offset], it[offset + 1])
    }

    /// first vertex address
    pub fn mesh_vertadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        usize::try_from(self.binding.mesh_vertadr()[id.index()]).ok()
    }
    /// number of vertices
    pub fn mesh_vertnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        self.binding.mesh_vertnum()[id.index()] as usize
    }
    /// first face address
    pub fn mesh_faceadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        usize::try_from(self.binding.mesh_faceadr()[id.index()]).ok()
    }
    /// number of faces
    pub fn mesh_facenum(&self, id: ObjectId<obj::Mesh>) -> usize {
        self.binding.mesh_facenum()[id.index()] as usize
    }
    /// address of bvh root
    pub fn mesh_bvhadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        usize::try_from(self.binding.mesh_bvhadr()[id.index()]).ok()
    }
    /// number of bounding volumes
    pub fn mesh_bvhnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        self.binding.mesh_bvhnum()[id.index()] as usize
    }
    /// first normal address
    pub fn mesh_normaladr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        usize::try_from(self.binding.mesh_normaladr()[id.index()]).ok()
    }
    /// number of normals
    pub fn mesh_normalnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        self.binding.mesh_normalnum()[id.index()] as usize
    }
    /// texcoord data address
    pub fn mesh_texcoordadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        usize::try_from(self.binding.mesh_texcoordadr()[id.index()]).ok()
    }
    /// number of texcoord
    pub fn mesh_texcoordnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        self.binding.mesh_texcoordnum()[id.index()] as usize
    }
    /// vertex positions for all meshes
    pub fn mesh_vert(&self, id: VertexId<obj::Mesh>) -> (f32, f32, f32) {
        let (it, offset) = (self.binding.mesh_vert(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// normals for all meshes
    pub fn mesh_normal(&self, id: NormalId) -> (f32, f32, f32) {
        let (it, offset) = (self.binding.mesh_normal(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// vertex texcoords for all meshes
    pub fn mesh_texcoord(&self, id: TexcoordId<obj::Mesh>) -> (f32, f32) {
        let (it, offset) = (self.binding.mesh_texcoord(), id.index() * 2);
        (it[offset], it[offset + 1])
    }
    /// vertex face data
    pub fn mesh_face(&self, id: FaceId<obj::Mesh>) -> (VertexId<obj::Mesh>, VertexId<obj::Mesh>, VertexId<obj::Mesh>) {
        let (it, offset) = (self.binding.mesh_face(), id.index() * 3);
        (
            unsafe { VertexId::new_unchecked(it[offset] as usize) },
            unsafe { VertexId::new_unchecked(it[offset + 1] as usize) },
            unsafe { VertexId::new_unchecked(it[offset + 2] as usize) }
        )
    }
    /// normal face data
    pub fn mesh_facenormal(&self, id: FaceId<obj::Mesh>) -> (NormalId, NormalId, NormalId) {
        let (it, offset) = (self.binding.mesh_facenormal(), id.index() * 3);
        (
            unsafe { NormalId::new_unchecked(it[offset] as usize) },
            unsafe { NormalId::new_unchecked(it[offset + 1] as usize) },
            unsafe { NormalId::new_unchecked(it[offset + 2] as usize) }
        )
    }
    /// texture face data
    pub fn mesh_facetexcoord(&self, id: FaceId<obj::Mesh>) -> (TexcoordId<obj::Mesh>, TexcoordId<obj::Mesh>, TexcoordId<obj::Mesh>) {
        let (it, offset) = (self.binding.mesh_facetexcoord(), id.index() * 3);
        (
            unsafe { TexcoordId::new_unchecked(it[offset] as usize) },
            unsafe { TexcoordId::new_unchecked(it[offset + 1] as usize) },
            unsafe { TexcoordId::new_unchecked(it[offset + 2] as usize) }
        )
    }
    /// scaling applied to asset vertices
    pub fn mesh_scale(&self, id: ObjectId<obj::Mesh>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.mesh_scale(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// translation applied to asset vertices
    pub fn mesh_pos(&self, id: ObjectId<obj::Mesh>) -> (f64, f64, f64) {
        let (it, offset) = (self.binding.mesh_pos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// rotation applied to asset vertices
    pub fn mesh_quat(&self, id: ObjectId<obj::Mesh>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.mesh_quat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// address of asset path for mesh
    pub fn mesh_pathadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        usize::try_from(self.binding.mesh_pathadr()[id.index()]).ok()
    }
    // TODO `mesh_poly*`

    /// skin material id
    pub fn skin_matid(&self, id: ObjectId<obj::Skin>) -> Option<ObjectId<obj::Material>> {
        let index = usize::try_from(self.binding.skin_matid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn skin_group(&self, id: ObjectId<obj::Skin>) -> i32 {
        self.binding.skin_group()[id.index()]
    }
    /// rgba when material is omitted
    pub fn skin_rgba(&self, id: ObjectId<obj::Skin>) -> Rgba {
        let (it, offset) = (self.binding.skin_rgba(), id.index() * 4);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: it[offset + 3] }
    }
    /// inflate skin in normal direction
    pub fn skin_inflate(&self, id: ObjectId<obj::Skin>) -> f32 {
        self.binding.skin_inflate()[id.index()]
    }
    /// first vertex address
    pub fn skin_vertadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        usize::try_from(self.binding.skin_vertadr()[id.index()]).ok()
    }
    /// number of vertices
    pub fn skin_vertnum(&self, id: ObjectId<obj::Skin>) -> usize {
        self.binding.skin_vertnum()[id.index()] as usize
    }
    /// texcoord data address
    pub fn skin_texcoordadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        usize::try_from(self.binding.skin_texcoordadr()[id.index()]).ok()
    }
    /// first face address
    pub fn skin_faceadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        usize::try_from(self.binding.skin_faceadr()[id.index()]).ok()
    }
    /// number of faces
    pub fn skin_facenum(&self, id: ObjectId<obj::Skin>) -> usize {
        self.binding.skin_facenum()[id.index()] as usize
    }
    /// first bone in skin
    pub fn skin_boneadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        usize::try_from(self.binding.skin_boneadr()[id.index()]).ok()
    }
    /// number of bones in skin
    pub fn skin_bonenum(&self, id: ObjectId<obj::Skin>) -> usize {
        self.binding.skin_bonenum()[id.index()] as usize
    }
    /// vertex positions for all skin meshes
    pub fn skin_vert(&self, id: VertexId<obj::Skin>) -> (f32, f32, f32) {
        let (it, offset) = (self.binding.skin_vert(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// vertex texcoords for all skin meshes
    pub fn skin_texcoord(&self, id: TexcoordId<obj::Skin>) -> (f32, f32) {
        let (it, offset) = (self.binding.skin_texcoord(), id.index() * 2);
        (it[offset], it[offset + 1])
    }
    /// triangle faces for all skin meshes
    pub fn skin_face(&self, id: FaceId<obj::Skin>) -> (VertexId<obj::Skin>, VertexId<obj::Skin>, VertexId<obj::Skin>) {
        let (it, offset) = (self.binding.skin_face(), id.index() * 3);
        (
            unsafe { VertexId::new_unchecked(it[offset] as usize) },
            unsafe { VertexId::new_unchecked(it[offset + 1] as usize) },
            unsafe { VertexId::new_unchecked(it[offset + 2] as usize) }
        )
    }
    /// first vertex in each bone
    pub fn skin_bonevertadr(&self, id: BoneId) -> Option<usize> {
        usize::try_from(self.binding.skin_bonevertadr()[id.index()]).ok()
    }
    /// number of vertices in each bone
    pub fn skin_bonevertnum(&self, id: BoneId) -> usize {
        self.binding.skin_bonevertnum()[id.index()] as usize
    }
    /// bind pos of each bone
    pub fn skin_bonebindpos(&self, id: BoneId) -> (f32, f32, f32) {
        let (it, offset) = (self.binding.skin_bonebindpos(), id.index() * 3);
        (it[offset], it[offset + 1], it[offset + 2])
    }
    /// bind quat of each bone
    pub fn skin_bonebindquat(&self, id: BoneId) -> (f32, f32, f32, f32) {
        let (it, offset) = (self.binding.skin_bonebindquat(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// body id of each bone
    pub fn skin_bonebodyid(&self, id: BoneId) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.skin_bonebodyid()[id.index()] as usize) }
    }
    /// mesh ids of vertices in each bone
    pub fn skin_bonevertid(&self, id: BoneVertexId) -> VertexId<obj::Skin> {
        unsafe { VertexId::new_unchecked(self.binding.skin_bonevertid()[id.index()] as usize) }
    }
    /// weights of vertices in each bone
    pub fn skin_bonevertweight(&self, id: BoneVertexId) -> f32 {
        self.binding.skin_bonevertweight()[id.index()]
    }
    /// address of asset path for skin
    pub fn skin_pathadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        usize::try_from(self.binding.skin_pathadr()[id.index()]).ok()
    }

    /// (x, y, z_top, z_bottom)
    pub fn hfield_size(&self, id: ObjectId<obj::HField>) -> (f64, f64, f64, f64) {
        let (it, offset) = (self.binding.hfield_size(), id.index() * 4);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3])
    }
    /// number of rows in grid
    pub fn hfield_nrow(&self, id: ObjectId<obj::HField>) -> usize {
        self.binding.hfield_nrow()[id.index()] as usize
    }
    /// number of columns in grid
    pub fn hfield_ncol(&self, id: ObjectId<obj::HField>) -> usize {
        self.binding.hfield_ncol()[id.index()] as usize
    }
    /// address in hfield_data
    pub fn hfield_adr(&self, id: ObjectId<obj::HField>) -> usize {
        self.binding.hfield_adr()[id.index()] as usize
    }
    /// elevation data
    pub fn hfield_data(&self, id: HFieldDataId) -> f32 {
        self.binding.hfield_data()[id.index()]
    }
    /// address of hfield asset path
    pub fn hfield_pathadr(&self, id: ObjectId<obj::HField>) -> Option<usize> {
        usize::try_from(self.binding.hfield_pathadr()[id.index()]).ok()
    }

    /// texture type (mjtTexture)
    pub fn tex_type(&self, id: ObjectId<obj::Texture>) -> mjtTexture {
        unsafe { tx(self.binding.tex_type()[id.index()] as u32) }
    }
    /// number of rows in texture image [px]
    pub fn tex_height(&self, id: ObjectId<obj::Texture>) -> usize {
        self.binding.tex_height()[id.index()] as usize
    }
    /// number of columns in texture image [px]
    pub fn tex_width(&self, id: ObjectId<obj::Texture>) -> usize {
        self.binding.tex_width()[id.index()] as usize
    }
    /// number of channels in texture image
    pub fn tex_nchannel(&self, id: ObjectId<obj::Texture>) -> usize {
        self.binding.tex_nchannel()[id.index()] as usize
    }
    /// start address in tex_data
    pub fn tex_adr(&self, id: ObjectId<obj::Texture>) -> usize {
        self.binding.tex_adr()[id.index()] as usize
    }
    /// pixel values
    pub fn tex_data(&self, id: TexDataId) -> u8 {
        self.binding.tex_data()[id.index()]
    }
    /// address of texture asset path
    pub fn tex_pathadr(&self, id: ObjectId<obj::Texture>) -> Option<usize> {
        usize::try_from(self.binding.tex_pathadr()[id.index()]).ok()
    }

    /// indices of textures
    pub fn mat_texid(&self, id: ObjectId<obj::Material>) -> [Option<ObjectId<obj::Texture>>; mjNTEXROLE] {
        let (it, offset) = (self.binding.mat_texid(), id.index() * mjNTEXROLE);
        std::array::from_fn(|i| {
            let index = usize::try_from(it[offset + i]).ok()?;
            Some(unsafe { ObjectId::new_unchecked(index) })
        })
    }
    /// make texture cube uniform
    pub fn mat_texuniform(&self, id: ObjectId<obj::Material>) -> bool {
        self.binding.mat_texuniform()[id.index()] != 0
    }
    /// texture repetition for 2d mapping
    pub fn mat_texrepeat(&self, id: ObjectId<obj::Material>) -> (f32, f32) {
        let (it, offset) = (self.binding.mat_texrepeat(), id.index() * 2);
        (it[offset], it[offset + 1])
    }
    /// emission (x rgb)
    pub fn mat_emission(&self, id: ObjectId<obj::Material>) -> f32 {
        self.binding.mat_emission()[id.index()]
    }
    /// specular (x white)
    pub fn mat_specular(&self, id: ObjectId<obj::Material>) -> f32 {
        self.binding.mat_specular()[id.index()]
    }
    /// shininess coef
    pub fn mat_shininess(&self, id: ObjectId<obj::Material>) -> f32 {
        self.binding.mat_shininess()[id.index()]
    }
    /// reflectance (0: disable)
    pub fn mat_reflectance(&self, id: ObjectId<obj::Material>) -> f32 {
        self.binding.mat_reflectance()[id.index()]
    }
    /// metallic coef
    pub fn mat_metallic(&self, id: ObjectId<obj::Material>) -> f32 {
        self.binding.mat_metallic()[id.index()]
    }
    /// roughness coef
    pub fn mat_roughness(&self, id: ObjectId<obj::Material>) -> f32 {
        self.binding.mat_roughness()[id.index()]
    }
    /// rgba
    pub fn mat_rgba(&self, id: ObjectId<obj::Material>) -> Rgba {
        let (it, offset) = (self.binding.mat_rgba(), id.index() * 4);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: it[offset + 3] }
    }

    /// contact dimensionality (1, 3, 4, 6)
    pub fn pair_dim(&self, id: ObjectId<obj::Pair>) -> usize {
        self.binding.pair_dim()[id.index()] as usize
    }
    /// id of geom1
    pub fn pair_geom1(&self, id: ObjectId<obj::Pair>) -> ObjectId<obj::Geom> {
        unsafe { ObjectId::new_unchecked(self.binding.pair_geom1()[id.index()] as usize) }
    }
    /// id of geom2
    pub fn pair_geom2(&self, id: ObjectId<obj::Pair>) -> ObjectId<obj::Geom> {
        unsafe { ObjectId::new_unchecked(self.binding.pair_geom2()[id.index()] as usize) }
    }
    /// body1 << 16 + body2
    pub fn pair_signature(&self, id: ObjectId<obj::Pair>) -> u32 {
        self.binding.pair_signature()[id.index()] as u32
    }
    /// solver reference: contact normal
    pub fn pair_solref(&self, id: ObjectId<obj::Pair>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.pair_solref(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// solver reference: contact friction
    pub fn pair_solreffriction(&self, id: ObjectId<obj::Pair>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.pair_solreffriction(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// solver impedance: contact
    pub fn pair_solimp(&self, id: ObjectId<obj::Pair>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.pair_solimp(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// detect contact if dist < margin
    pub fn pair_margin(&self, id: ObjectId<obj::Pair>) -> f64 {
        self.binding.pair_margin()[id.index()]
    }
    /// include in solver if dist < margin - gap
    pub fn pair_gap(&self, id: ObjectId<obj::Pair>) -> f64 {
        self.binding.pair_gap()[id.index()]
    }
    /// friction for (tangent1, tangent2, spin, roll1, roll2)
    pub fn pair_friction(&self, id: ObjectId<obj::Pair>) -> (f64, f64, f64, f64, f64) {
        let (it, offset) = (self.binding.pair_friction(), id.index() * 5);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4])
    }

    /// body1 << 16 + body2
    pub fn exclude_signature(&self, id: ObjectId<obj::Exclude>) -> u32 {
        self.binding.exclude_signature()[id.index()] as u32
    }

    /// constraint type (mjtEq)
    pub fn eq_type(&self, id: ObjectId<obj::Equality>) -> mjtEq {
        unsafe { tx(self.binding.eq_type()[id.index()] as u32) }
    }
    /// get object ids as `ObjectId<O>`; `None` if object is not of type `O`.
    /// 
    /// the object type can be got (as value) with `.eq_objtype()`.
    pub fn eq_objid<O: Obj>(&self, id: ObjectId<obj::Equality>) -> Option<(ObjectId<O>, ObjectId<O>)> {
        if self.binding.eq_objtype()[id.index()] as u32 != unsafe { tx(O::TYPE) } {return None}
        Some((
            unsafe { ObjectId::new_unchecked(self.binding.eq_obj1id()[id.index()] as usize) },
            unsafe { ObjectId::new_unchecked(self.binding.eq_obj2id()[id.index()] as usize) }
        ))
    }
    /// type of both objects (mjtObj)
    pub fn eq_objtype(&self, id: ObjectId<obj::Equality>) -> mjtObj {
        unsafe { tx(self.binding.eq_objtype()[id.index()] as u32) }
    }
    /// initial enable/disable constraint state
    pub fn eq_active0(&self, id: ObjectId<obj::Equality>) -> bool {
        self.binding.eq_active0()[id.index()] != 0
    }
    /// constraint solver reference
    pub fn eq_solref(&self, id: ObjectId<obj::Equality>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.eq_solref(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance
    pub fn eq_solimp(&self, id: ObjectId<obj::Equality>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.eq_solimp(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// numeric data for constraint
    pub fn eq_data(&self, id: ObjectId<obj::Equality>) -> [f64; mjNEQDATA] {
        let (it, offset) = (self.binding.eq_data(), id.index() * mjNEQDATA);
        std::array::from_fn(|i| it[offset + i])
    }

    /// first address of tendon path
    pub fn tendon_adr(&self, id: ObjectId<obj::Tendon>) -> usize {
        self.binding.tendon_adr()[id.index()] as usize
    }
    /// number of objects in tendon path
    pub fn tendon_num(&self, id: ObjectId<obj::Tendon>) -> usize {
        self.binding.tendon_num()[id.index()] as usize
    }
    /// material id for rendering
    pub fn tendon_matid(&self, id: ObjectId<obj::Tendon>) -> Option<ObjectId<obj::Material>> {
        let index = usize::try_from(self.binding.tendon_matid()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn tendon_group(&self, id: ObjectId<obj::Tendon>) -> i32 {
        self.binding.tendon_group()[id.index()]
    }
    /// does tendon have length limits
    pub fn tendon_limited(&self, id: ObjectId<obj::Tendon>) -> bool {
        self.binding.tendon_limited()[id.index()] != 0
    }
    /// does tendon have actuator force limits
    pub fn tendon_actfrclimited(&self, id: ObjectId<obj::Tendon>) -> bool {
        self.binding.tendon_actfrclimited()[id.index()] != 0
    }
    /// width for rendering
    pub fn tendon_width(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_width()[id.index()]
    }
    /// constraint solver reference: limit
    pub fn tendon_solref_lim(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.tendon_solref_lim(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance: limit
    pub fn tendon_solimp_lim(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.tendon_solimp_lim(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// constraint solver reference: friction
    pub fn tendon_solref_fri(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNREF] {
        let (it, offset) = (self.binding.tendon_solref_fri(), id.index() * mjNREF);
        [it[offset], it[offset + 1]]
    }
    /// constraint solver impedance: friction
    pub fn tendon_solimp_fri(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNIMP] {
        let (it, offset) = (self.binding.tendon_solimp_fri(), id.index() * mjNIMP);
        [it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4]]
    }
    /// tendon length limits
    pub fn tendon_range(&self, id: ObjectId<obj::Tendon>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.tendon_range(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// range of total actuator force
    pub fn tendon_actfrcrange(&self, id: ObjectId<obj::Tendon>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.tendon_actfrcrange(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// min distance for limit detection
    pub fn tendon_margin(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_margin()[id.index()]
    }
    /// stiffness coefficient
    pub fn tendon_stiffness(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_stiffness()[id.index()]
    }
    /// damping coefficient
    pub fn tendon_damping(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_damping()[id.index()]
    }
    /// inertia associated with tendon velocity
    pub fn tendon_armature(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_armature()[id.index()]
    }
    /// loss due to friction
    pub fn tendon_frictionloss(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_frictionloss()[id.index()]
    }
    /// spring resting length range
    pub fn tendon_lengthspring(&self, id: ObjectId<obj::Tendon>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.tendon_lengthspring(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// tendon length in qpos0
    pub fn tendon_length0(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_length0()[id.index()]
    }
    /// edge inv. weight in qpos0
    pub fn tendon_invweight0(&self, id: ObjectId<obj::Tendon>) -> f64 {
        self.binding.tendon_invweight0()[id.index()]
    }
    /// rgba when material is omitted
    pub fn tendon_rgba(&self, id: ObjectId<obj::Tendon>) -> Rgba {
        let (it, offset) = (self.binding.tendon_rgba(), id.index() * 4);
        Rgba { r: it[offset], g: it[offset + 1], b: it[offset + 2], a: it[offset + 3] }
    }

    /// wrap object type (mjtWrap)
    pub fn wrap_type(&self, index: usize) -> mjtWrap {
        unsafe { tx(self.binding.wrap_type()[index] as u32) }
    }
    /// object id: geom, site, joint
    /// 
    /// Returns `None` if specified object type `O` does not match the wrap object type.
    /// The wrap object type can be got (as value) with `.wrap_type(index)`.
    pub fn wrap_objid<O: Obj>(&self, index: usize) -> Option<ObjectId<O>> {
        if self.binding.wrap_type()[index] as u32 != unsafe { tx(O::TYPE) } {return None}
        Some(unsafe { ObjectId::new_unchecked(self.binding.wrap_objid()[index] as usize) })
    }
    /// divisor, joint coef, or site id
    pub fn wrap_prm(&self, index: usize) -> f64 {
        self.binding.wrap_prm()[index]
    }
    
    /// transmission type (mjtTrn)
    pub fn actuator_trntype(&self, id: ObjectId<obj::Actuator>) -> mjtTrn {
        unsafe { tx(self.binding.actuator_trntype()[id.index()] as u32) }
    }
    /// dynamics type (mjtDyn)
    pub fn actuator_dyntype(&self, id: ObjectId<obj::Actuator>) -> mjtDyn {
        unsafe { tx(self.binding.actuator_dyntype()[id.index()] as u32) }
    }
    /// gain type (mjtGain)
    pub fn actuator_gaintype(&self, id: ObjectId<obj::Actuator>) -> mjtGain {
        unsafe { tx(self.binding.actuator_gaintype()[id.index()] as u32) }
    }
    /// bias type (mjtBias)
    pub fn actuator_biastype(&self, id: ObjectId<obj::Actuator>) -> mjtBias {
        unsafe { tx(self.binding.actuator_biastype()[id.index()] as u32) }
    }
    /// transmission id: joint, tendon, site
    ///
    /// Returns `None` if the transmission id does not match the object type `O`.
    pub fn actuator_trnid<O: Obj>(&self, id: ObjectId<obj::Actuator>) -> (Option<ObjectId<O>>, Option<ObjectId<O>>) {
        let index = id.index() * 2;
        if self.binding.actuator_trnid()[index] as u32 != unsafe { tx(O::TYPE) } {return (None, None)}
        (
            Some(unsafe { ObjectId::new_unchecked(self.binding.actuator_trnid()[index] as usize) }),
            Some(unsafe { ObjectId::new_unchecked(self.binding.actuator_trnid()[index + 1] as usize) })
        )
    }
    /// first activation address; None: stateless
    pub fn actuator_actadr(&self, id: ObjectId<obj::Actuator>) -> Option<usize> {
        usize::try_from(self.binding.actuator_actadr()[id.index()]).ok()
    }
    /// number of activation variables
    pub fn actuator_actnum(&self, id: ObjectId<obj::Actuator>) -> usize {
        self.binding.actuator_actnum()[id.index()] as usize
    }
    /// group for visibility
    pub fn actuator_group(&self, id: ObjectId<obj::Actuator>) -> i32 {
        self.binding.actuator_group()[id.index()]
    }
    /// is control limited
    pub fn actuator_ctrllimited(&self, id: ObjectId<obj::Actuator>) -> bool {
        self.binding.actuator_ctrllimited()[id.index()] != 0
    }
    /// is force limited
    pub fn actuator_forcelimited(&self, id: ObjectId<obj::Actuator>) -> bool {
        self.binding.actuator_forcelimited()[id.index()] != 0
    }
    /// is activation limited
    pub fn actuator_actlimited(&self, id: ObjectId<obj::Actuator>) -> bool {
        self.binding.actuator_actlimited()[id.index()] != 0
    }
    /// dynamics parameters
    pub fn actuator_dynprm(&self, id: ObjectId<obj::Actuator>) -> [f64; mjNDYN] {
        let (it, offset) = (self.binding.actuator_dynprm(), id.index() * mjNDYN);
        std::array::from_fn(|i| it[offset + i])
    }
    /// gain parameters
    pub fn actuator_gainprm(&self, id: ObjectId<obj::Actuator>) -> [f64; mjNGAIN] {
        let (it, offset) = (self.binding.actuator_gainprm(), id.index() * mjNGAIN);
        std::array::from_fn(|i| it[offset + i])
    }
    /// bias parameters
    pub fn actuator_biasprm(&self, id: ObjectId<obj::Actuator>) -> [f64; mjNBIAS] {
        let (it, offset) = (self.binding.actuator_biasprm(), id.index() * mjNBIAS);
        std::array::from_fn(|i| it[offset + i])
    }
    /// step activation before force
    pub fn actuator_actearly(&self, id: ObjectId<obj::Actuator>) -> bool {
        self.binding.actuator_actearly()[id.index()] != 0
    }
    /// range of controls
    pub fn actuator_ctrlrange(&self, id: ObjectId<obj::Actuator>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.actuator_ctrlrange(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// range of forces
    pub fn actuator_forcerange(&self, id: ObjectId<obj::Actuator>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.actuator_forcerange(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// range of activations
    pub fn actuator_actrange(&self, id: ObjectId<obj::Actuator>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.actuator_actrange(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// scale length and transmitted force
    pub fn actuator_gear(&self, id: ObjectId<obj::Actuator>) -> (f64, f64, f64, f64, f64, f64) {
        let (it, offset) = (self.binding.actuator_gear(), id.index() * 6);
        (it[offset], it[offset + 1], it[offset + 2], it[offset + 3], it[offset + 4], it[offset + 5])
    }
    /// crank length for slider-crank
    pub fn actuator_cranklength(&self, id: ObjectId<obj::Actuator>) -> f64 {
        self.binding.actuator_cranklength()[id.index()]
    }
    /// acceleration from unit force in qpos0
    pub fn actuator_acc0(&self, id: ObjectId<obj::Actuator>) -> f64 {
        self.binding.actuator_acc0()[id.index()]
    }
    /// actuator length in qpos0
    pub fn actuator_length0(&self, id: ObjectId<obj::Actuator>) -> f64 {
        self.binding.actuator_length0()[id.index()]
    }
    /// feasible actuator length range
    pub fn actuator_lengthrange(&self, id: ObjectId<obj::Actuator>) -> impl std::ops::RangeBounds<f64> {
        let (it, offset) = (self.binding.actuator_lengthrange(), id.index() * 2);
        it[offset]..=it[offset + 1]
    }
    /// plugin instance id; None: not a plugin
    pub fn actuator_plugin(&self, id: ObjectId<obj::Actuator>) -> Option<ObjectId<obj::Plugin>> {
        let index = usize::try_from(self.binding.actuator_plugin()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }

    /// sendor type (mjtSensor)
    pub fn sensor_type(&self, id: ObjectId<obj::Sensor>) -> mjtSensor {
        unsafe { tx(self.binding.sensor_type()[id.index()] as u32) }
    }
    /// numeric data type (mjtDataType)
    pub fn sensor_datatype(&self, id: ObjectId<obj::Sensor>) -> mjtDataType {
        unsafe { tx(self.binding.sensor_datatype()[id.index()] as u32) }
    }
    /// required compute stage (mjtStage)
    pub fn sensor_needstage(&self, id: ObjectId<obj::Sensor>) -> mjtStage {
        unsafe { tx(self.binding.sensor_needstage()[id.index()] as u32) }
    }
    /// type of sensorized object (mjtObj)
    pub fn sensor_objtype(&self, id: ObjectId<obj::Sensor>) -> mjtObj {
        unsafe { tx(self.binding.sensor_objtype()[id.index()] as u32) }
    }
    /// id of sensorized object
    ///
    /// Returns `None` if the object type does not match the sensor object type.
    /// The sensor object type can be got (as value) with `.sensor_objtype(id)`.
    pub fn sensor_objid<O: Obj>(&self, id: ObjectId<obj::Sensor>) -> Option<ObjectId<O>> {
        if self.binding.sensor_objtype()[id.index()] as u32 != unsafe { tx(O::TYPE) } {return None}
        Some(unsafe { ObjectId::new_unchecked(self.binding.sensor_objid()[id.index()] as usize) })
    }
    /// type of reference frame (mjtObj)
    pub fn sensor_reftype(&self, id: ObjectId<obj::Sensor>) -> mjtObj {
        unsafe { tx(self.binding.sensor_reftype()[id.index()] as u32) }
    }
    /// id of reference frame;
    /// 
    /// Returns `None` when it's global frame or, if the reference frame type does not match the sensor reference type.
    /// The sensor reference type can be got (as value) with `.sensor_reftype(id)`.
    pub fn sensor_refid<O: Obj>(&self, id: ObjectId<obj::Sensor>) -> Option<ObjectId<O>> {
        if self.binding.sensor_refid()[id.index()] == -1 {return None}
        if self.binding.sensor_reftype()[id.index()] as u32 != unsafe { tx(O::TYPE) } {return None}
        Some(unsafe { ObjectId::new_unchecked(self.binding.sensor_refid()[id.index()] as usize) })
    }
    /// number of scalar outputs
    pub fn sensor_dim(&self, id: ObjectId<obj::Sensor>) -> usize {
        self.binding.sensor_dim()[id.index()] as usize
    }
    /// address in sensor array
    pub fn sensor_adr(&self, id: ObjectId<obj::Sensor>) -> usize {
        self.binding.sensor_adr()[id.index()] as usize
    }
    /// cutoff for real and positive; 0: ignore
    pub fn sensor_cutoff(&self, id: ObjectId<obj::Sensor>) -> f64 {
        self.binding.sensor_cutoff()[id.index()]
    }
    /// noise standard deviation
    pub fn sensor_noise(&self, id: ObjectId<obj::Sensor>) -> f64 {
        self.binding.sensor_noise()[id.index()]
    }
    /// plugin instance id; None: not a plugin
    pub fn sensor_plugin(&self, id: ObjectId<obj::Sensor>) -> Option<ObjectId<obj::Plugin>> {
        let index = usize::try_from(self.binding.sensor_plugin()[id.index()]).ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }

    /// globally registered plugin slot number
    pub fn plugin(&self, id: ObjectId<obj::Plugin>) -> usize {
        self.binding.plugin()[id.index()] as usize
    }
    /// address in the plugin state array
    pub fn plugin_stateadr(&self, id: ObjectId<obj::Plugin>) -> Option<usize> {
        usize::try_from(self.binding.plugin_stateadr()[id.index()]).ok()
    }
    /// number of states in the plugin instance
    pub fn plugin_statenum(&self, id: ObjectId<obj::Plugin>) -> usize {
        self.binding.plugin_statenum()[id.index()] as usize
    }
    /// config attributes of plugin instances
    pub fn plugin_attr(&self, id: ObjectId<obj::Plugin>) -> *const std::ffi::c_char {
        (&self.binding.plugin_attr()[id.index()]) as *const std::ffi::c_char
    }
    /// address to each instance's config attribute
    pub fn plugin_attradr(&self, id: ObjectId<obj::Plugin>) -> Option<usize> {
        usize::try_from(self.binding.plugin_attradr()[id.index()]).ok()
    }

    /// keyframe time
    pub fn key_time(&self, id: ObjectId<obj::Key>) -> f64 {
        self.binding.key_time()[id.index()]
    }
    /// keyframe position: f64 * `nq`
    pub fn key_qpos(&self, id: ObjectId<obj::Key>) -> &[f64] {
        let offset = id.index() * self.binding.nq();
        &self.binding.key_qpos()[offset..offset + self.binding.nq()]
    }
    /// keyframe velocity: f64 * `nv`
    pub fn key_qvel(&self, id: ObjectId<obj::Key>) -> &[f64] {
        let offset = id.index() * self.binding.nv();
        &self.binding.key_qvel()[offset..offset + self.binding.nv()]
    }
    /// keyframe activation: f64 * `na`
    pub fn key_act(&self, id: ObjectId<obj::Key>) -> &[f64] {
        let offset = id.index() * self.binding.na();
        &self.binding.key_act()[offset..offset + self.binding.na()]
    }
    /// keyframe mocap position: f64 * `nmocap * 3`
    pub fn key_mpos(&self, id: ObjectId<obj::Key>) -> &[f64] {
        let offset = id.index() * self.binding.nmocap() * 3;
        &self.binding.key_mpos()[offset..offset + self.binding.nmocap() * 3]
    }
    /// keyframe mocap quaternion: f64 * `nmocap * 4`
    pub fn key_mquat(&self, id: ObjectId<obj::Key>) -> &[f64] {
        let offset = id.index() * self.binding.nmocap() * 4;
        &self.binding.key_mquat()[offset..offset + self.binding.nmocap() * 4]
    }
    /// keyframe control: f64 * `nu`
    pub fn key_ctrl(&self, id: ObjectId<obj::Key>) -> &[f64] {
        let offset = id.index() * self.binding.nu();
        &self.binding.key_ctrl()[offset..offset + self.binding.nu()]
    }
/* */
}
