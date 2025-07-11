use acrobot_qtable::*;
use qtable::strategy;
use oxide_control::TimeStep;

fn main() {
    let mut args = std::env::args().skip(1);
    let target_qtable_path = args.next().expect("Usage: simulate <target_qtable_path>");

    let t = TrainedAgent::load(&target_qtable_path).expect("Failed to load trained agent");

    let mut env = oxide_control::Environment::new::<AcrobotAction>(
        Acrobot::new(),
        AcrobotBalanceTask {
            do_swing: false,
            discount: 0.99,
            n_arm_digitization: t.n_arm_digitization(),
            n_pendulum_digitization: t.n_pendulum_digitization(),
        },
    );

    let mut obs = env.reset();
    loop {
        /* TODO: render the physics */

        let state = env.task().state(&obs);
        let action = t.get_action::<strategy::MostQValue>(state);
        match env.step(action) {
            TimeStep::Step { observation, .. } => {
                obs = observation;
            }
            TimeStep::Finish { .. } => {
                break;
            }
        }
    }
}
