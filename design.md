# `oxide_control`, the `dm_control` layer for Rust

* `struct Environment<T: Task, O: Observation<Phyics = T::Physics>, A: Action<Physics: T::Physics>>`<!-- A: Agent<Physics = T::Physics>>`
  * `phyics: T::Physics` : a specific physics defined by the corresponded XML
  * `task: T` : a rulebook of what to do on the physics
  <!-- * `agent: A` : an agent that learns on the physcis -->
  * `rewarder: Box<dyn Fn(&O) -> f64>`
  * `discount: f64`
  * `fn reset(&mut self) -> O`
  * `fn step(&mut self, A) -> (O, Option<f64>)`

* `trait Physics: Deref/DerefMut<Target = RawPhysics>`
  * `xml` : the XML that defines this physics
  * `fn derive(RawPhyics) -> Self` : construct this specific physics from `RawPhysics`, a direct wrapper of MuJoCo physics

* `trait Task`
  * `type Physics: Physics`
  * `fn init_episode(&self, &mut Self::Physics)`
  * `fn should_finish_episode(&self, &Self::Physics) -> bool`

<!--
* `trait Agent`
  * `type Physics: Physics`
  * `type Observation: Observation<Physics = Self::Physics>`
  * `type Action: Action<Physics = Self::Physics>`
  * `fn policy(&self, Self::Observation) -> Self::Action`
  * `fn learn(&mut self, Self::Observation, reward: f64)`
-->

* `trait Observation`
  * `type Physics: Physics`
  * `fn generate(&Self::Physics) -> Self`

* `trait Action`
  * `type Physics: Physics`
  * `fn serialize(self) -> impl AsRef<[f64]>` : convert the fully-typed action into actual control input for the actuators of `Self::Physics` (the size of returned vec must be `Self::Physics`'s model's `nu`)
