use acrobot_qtable::*;
use qtable::strategy;
use oxide_control::TimeStep;

fn get_reward(
    balance: &AcrobotBalanceTask,
    observation: &AcrobotObservation,
    physics: &Acrobot,
) -> f64 {
    let state = balance.state(observation);

    if oxide_control::Task::should_finish_episode(balance, observation, physics) {
        return -2000.0; // Penalty for finishing the episode
    }
    
    let best_n_pendulum_rad = balance.n_pendulum_digitization / 2;
    let good_n_pendulum_rad_min = best_n_pendulum_rad - (balance.n_pendulum_digitization / 4);
    let good_n_pendulum_rad_max = best_n_pendulum_rad + (balance.n_pendulum_digitization / 4);

    let best_n_arm_rad = balance.n_arm_digitization / 2;
    let good_n_arm_rad_min = best_n_arm_rad - (balance.n_arm_digitization / 4);
    let good_n_arm_rad_max = best_n_arm_rad + (balance.n_arm_digitization / 4);

    let (n_pend_rad, n_arm_rad) = (state.n_pendulum_rad, state.n_arm_rad);
    let pend_reward = if n_pend_rad == best_n_pendulum_rad {
        2.0
    } else if (best_n_pendulum_rad - 1..=best_n_pendulum_rad + 1).contains(&n_pend_rad) {
        1.5
    } else if (good_n_pendulum_rad_min..=good_n_pendulum_rad_max).contains(&n_pend_rad) {
        1.0 - (n_pend_rad as f64 - best_n_pendulum_rad as f64).abs() / (good_n_pendulum_rad_max - best_n_pendulum_rad) as f64
    } else {
        - 2.0 * (n_pend_rad as f64 - best_n_pendulum_rad as f64).abs() / (balance.n_pendulum_digitization as f64)
    };
    let arm_reward = if n_arm_rad == best_n_arm_rad {
        2.0
    } else if (best_n_arm_rad - 1..=best_n_arm_rad + 1).contains(&n_arm_rad) {
        1.5
    } else if (good_n_arm_rad_min..=good_n_arm_rad_max).contains(&n_arm_rad) {
        1.0 - (n_arm_rad as f64 - best_n_arm_rad as f64).abs() / (good_n_arm_rad_max - best_n_arm_rad) as f64
    } else {
        - 2.0 * (n_arm_rad as f64 - best_n_arm_rad as f64).abs() / (balance.n_arm_digitization as f64)
    };
    pend_reward * 2.0 + arm_reward
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

    let mut env = oxide_control::Environment::new::<AcrobotAction>(
        Acrobot::new(),
        AcrobotBalanceTask { n_arm_digitization, n_pendulum_digitization, get_reward },
    );

    let mut agent = {
        let config = QTableAgentConfig {
            action_size: 5,
            state_size: (n_arm_digitization.pow(2) * n_pendulum_digitization.pow(2)),
            initial_alpha: 0.5,
            initial_epsilon: 0.5,
        };
        match model_restore_file {
            None => QTableAgent::new(&config, &env),
            Some(path) => QTableAgent::load(&path)
                .expect(&format!("Failed to resotore agent from Q-table file `{}`", path.display())),
        }
    };

    // initialize the qtable by randomly exploring the environment
    for _ in 0..100 {/* warmup */
        let mut obs = env.reset();
        for _ in 0..400 {/* warmup step */
            let state = env.task().state(&obs);
            let action = agent.get_action::<strategy::Random>(state);
            match env.step(action) {
                TimeStep::Step { observation, reward, discount: _ } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    obs = observation;
                }
                TimeStep::Finish { observation, reward } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    break; // End the episode
                }
            }
        }
    }

    // main training loop
    for episode in 0..max_episodes {/* episode */
        let start_time = std::time::Instant::now();
        let mut episode_reward = 0.0;
        let mut obs = env.reset();

        for _ in 0..episode_length {/* step */
            let state = env.task().state(&obs);
            let action = agent.get_action::<strategy::EpsilonGreedy>(state);
            match env.step(action) {
                TimeStep::Step { observation, reward, discount:_ } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    episode_reward += reward;
                    obs = observation;
                }
                TimeStep::Finish { observation, reward } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    break; // End the episode
                }
            }
        }

        if episode > 0 && episode % model_log_interval == 0 {
            println!("[episode {episode}]: return: {:.2}, time: {:?}", episode_reward, start_time.elapsed());
            agent
                .save(model_log_directory.join(format!("agent-{episode}.json")))
                .expect("Failed to save Q-table");
        }

        agent.decay_alpha_with_rate(0.9999);
        agent.decay_epsilon_with_rate(0.9999);
    }
}
