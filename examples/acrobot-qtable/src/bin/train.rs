use acrobot_qtable::*;
use oxide_control::TimeStep;
use qtable::strategy;

/// AcrobotObservation をそのまま使えば連続的な物理量を扱えるが、
/// 元の Python コードベースを尊重して離散的な AcrobotState を用いている
fn get_reward(task: &AcrobotBalanceTask, state: &AcrobotState, action: &AcrobotAction) -> f64 {
    if task.should_finish_episode(state) {
        return -2000.0; // Penalty for finishing the episode
    }

    let position_reward = {
        const MAX_REWARD: f64 = 10.0;
        let pendulum_pos_error =
            (task.n_pendulum_digitization as f64 / 2.0 - state.n_pendulum_rad as f64).powi(2);
        let arm_pos_error = (task.n_arm_digitization as f64 / 2.0 - state.n_arm_rad as f64).powi(2);
        MAX_REWARD - (0.2 * pendulum_pos_error + 0.1 * arm_pos_error)
    };

    let velocity_penalty = {
        let pendulum_vel_error =
            (task.n_pendulum_digitization as f64 / 2.0 - state.n_pendulum_vel as f64).powi(2);
        let arm_vel_error =
            (task.n_pendulum_digitization as f64 / 2.0 - state.n_arm_vel as f64).powi(2);
        0.1 * pendulum_vel_error + 0.1 * arm_vel_error
    };

    let action_cost =
        { 0.02 * (task.action_size as f64 / 2.0 - action.digitization_index as f64).abs() };

    position_reward - velocity_penalty - action_cost
}

fn main() {
    macro_rules! env_config {
        ($( $config:ident: $T:ty = @$env:literal $(|| $default:expr)?; )*) => {
            $(
                let $config = {
                    let config = std::env::var($env).ok().map(
                        |s| s.trim().parse::<$T>().expect(&format!("Failed to parse {}", $env))
                    );
                    $(
                        let config = config.unwrap_or($default);
                    )?
                    config
                };
            )*
        };
    }
    env_config! {
        action_size: usize = @"ACTION_SIZE" || 5;
        n_arm_digitization: usize = @"N_ARM_DIGITIZATION" || 15;
        n_pendulum_digitization: usize = @"N_PENDULUM_DIGITIZATION" || 16;
        max_episodes: usize = @"MAX_EPISODES" || 1000000;
        episode_length: usize = @"EPISODE_LENGTH" || 5000;
        model_log_interval: usize = @"MODEL_LOG_INTERVAL" || 2000;
        model_restore_file: std::path::PathBuf = @"MODEL_RESTORE_FILE";
        model_log_directory: std::path::PathBuf = @"MODEL_LOG_DIRECTORY" || std::env::current_dir().unwrap()
            .join("models")
            .join(chrono::Local::now().format("%m-%d-%H-%M-%S").to_string());
    }

    std::fs::create_dir_all(&model_log_directory).expect("Failed to create model log directory");

    let mut env = oxide_control::Environment::new(
        Acrobot::new(),
        AcrobotBalanceTask {
            action_size,
            n_arm_digitization,
            n_pendulum_digitization,
            get_reward,
        },
    );

    let mut agent = {
        let config = QTableAgentConfig {
            action_size,
            state_size: (n_arm_digitization.pow(2) * n_pendulum_digitization.pow(2)),
            initial_alpha: 0.5,
            initial_epsilon: 0.5,
        };
        match model_restore_file {
            None => QTableAgent::new(&config, &env),
            Some(path) => QTableAgent::load(&path).expect(&format!(
                "Failed to resotore agent from Q-table file `{}`",
                path.display()
            )),
        }
    };

    // initialize the qtable by randomly exploring the environment
    for _ in 0..100 {
        /* warmup */
        let mut obs = env.reset();
        for _ in 0..400 {
            /* warmup step */
            let state = env.task().state(&obs);
            let action = agent.get_action::<strategy::Random>(state);
            match env.step(action) {
                TimeStep::Step {
                    observation,
                    reward,
                    discount: _,
                } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    obs = observation;
                }
                TimeStep::Finish {
                    observation,
                    reward,
                } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    break; // End the episode
                }
            }
        }
    }

    // main training loop
    for episode in 1..=max_episodes {
        /* episode */
        let start_time = std::time::Instant::now();
        let mut episode_reward = 0.0;
        let mut obs = env.reset();

        for _ in 0..episode_length {
            /* step */
            let state = env.task().state(&obs);
            let action = agent.get_action::<strategy::EpsilonGreedy>(state);
            match env.step(action) {
                TimeStep::Step {
                    observation,
                    reward,
                    discount: _,
                } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    episode_reward += reward;
                    obs = observation;
                }
                TimeStep::Finish {
                    observation,
                    reward,
                } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    break; // End the episode
                }
            }
        }

        if episode > 0 && episode % model_log_interval == 0 {
            println!(
                "[episode {episode}]: return: {:.2}, time: {:?}",
                episode_reward,
                start_time.elapsed()
            );
            agent
                .save(model_log_directory.join(format!("agent-{episode}.json")))
                .expect("Failed to save Q-table");
        }

        agent.decay_alpha_with_rate(0.9999);
        agent.decay_epsilon_with_rate(0.9999);
    }
}
