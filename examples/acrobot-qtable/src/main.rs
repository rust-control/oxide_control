use oxide_control::{Environment, Observation, Action, Task};
use qtable::{QTable, QUpdate, strategy};

struct Agent {
    qtable: QTable,
}

impl Agent {
    fn new() -> Self {
        Self {
            qtable: QTable::new(),
        }
    }

    fn get_action_by_softmax(
        &self,
        observation: &Observation,
    )
}

fn main() {
    println!("Hello, world!");
}
