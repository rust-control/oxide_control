use super::binding::{ObjectId, obj};

pub struct Model { pub binding: super::binding::mjModel }

impl Model {
    /* ObjectId -> {single value} */

    pub fn body_parentid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.body_parentid()[id.index()] as usize) }
    }

    pub fn body_rootid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        unsafe { ObjectId::new_unchecked(self.binding.body_rootid()[id.index()] as usize) }
    }

    /*
        :
        :
    */

    /* ObjectId -> ({value0}, {value1}) */

    pub fn body_invweight0(&self, id: ObjectId<obj::Body>) -> (f64, f64) {
        (self.binding.body_invweight0()[id.index()], self.binding.body_invweight0()[id.index() + 1])
    }

    /* ObjectId -> ({value0}, {value1}, {value2}) */

    pub fn body_pos(&self, id: ObjectId<obj::Body>) -> (f64, f64, f64) {
        (
            self.binding.body_pos()[id.index()],
            self.binding.body_pos()[id.index() + 1],
            self.binding.body_pos()[id.index() + 2],
        )
    }

    /*
        :
        :
    */
}
