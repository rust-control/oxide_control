use acrobot_qtable::*;
use oxide_control::TimeStep;
use qtable::strategy;

/// AcrobotObservation をそのまま使えば連続的な物理量を扱えるが、
/// 元の Python コードベースを尊重して離散的な AcrobotState を用いている
fn get_reward(task: &AcrobotBalanceTask, state: &AcrobotState, action: &AcrobotAction) -> f64 {
    if task.should_finish_episode(state) {
        return -2000.0;
    }

    let pend_pos_center = task.n_pendulum_digitization as f64 / 2.0;
    let arm_pos_center = task.n_arm_digitization as f64 / 2.0;
    let pend_vel_center = task.n_pendulum_digitization as f64 / 2.0;
    let arm_vel_center = task.n_arm_digitization as f64 / 2.0;

    if (state.n_pendulum_rad as f64 - pend_pos_center).abs() < 1.0
        && (state.n_arm_rad as f64 - arm_pos_center).abs() < 1.0
        && (state.n_pendulum_vel as f64 - pend_vel_center).abs() < 1.0
        && (state.n_arm_vel as f64 - arm_vel_center).abs() < 1.0
    {
        return 500.0;
    }

    let position_reward = {
        const MAX_REWARD: f64 = 10.0;
        let pendulum_pos_error = (pend_pos_center - state.n_pendulum_rad as f64).powi(2);
        let arm_pos_error = (arm_pos_center - state.n_arm_rad as f64).powi(2);
        MAX_REWARD - (0.2 * pendulum_pos_error + 0.1 * arm_pos_error)
    };

    let velocity_penalty = {
        let pendulum_vel_error = (pend_vel_center - state.n_pendulum_vel as f64).powi(2);
        let arm_vel_error = (arm_vel_center - state.n_arm_vel as f64).powi(2);
        0.001 * pendulum_vel_error + 0.002 * arm_vel_error
    };

    // この場合 action は「何もしない」真ん中が設定されていることを想定している。
    // よって task.action_size は奇数を前提として、真ん中の index のときに
    // action_cost が厳密に 0 になるようにしている
    let action_cost = {
        let center_action_index = (task.action_size - 1) as f64 / 2.0;
        0.01 * (center_action_index - action.digitization_index as f64).abs()
    };

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
        episode_length: usize = @"EPISODE_LENGTH" || 6000;
        model_save_interval: usize = @"MODEL_LOG_INTERVAL" || 10000;
        model_restore_file: std::path::PathBuf = @"MODEL_RESTORE_FILE";
        model_save_directory: std::path::PathBuf = @"MODEL_LOG_DIRECTORY" || std::env::current_dir().unwrap()
            .join("models")
            .join(chrono::Local::now().format("%m-%d-%H-%M-%S").to_string());
    }

    std::fs::create_dir_all(&model_save_directory).expect("Failed to create model log directory");

    print!("----> loading environment and agent...");
    std::io::Write::flush(&mut std::io::stdout()).unwrap();

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
            initial_alpha: 0.1,
            initial_epsilon: 0.5,
        };
        match &model_restore_file {
            None => QTableAgent::new(&config, &env),
            Some(path) => QTableAgent::load(&path).expect(&format!("Failed to resotore agent from Q-table file `{}`", path.display())),
        }
    };

    println!(" ✅loaded");

    if model_restore_file.is_none() {
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
                    TimeStep::Finish { observation, reward } => {
                        let next_state = env.task().state(&observation);
                        agent.learn(state, action, reward, next_state);
                        break; // End the episode
                    }
                }
            }
        }
    }

    // main training loop
    for episode in 1..=max_episodes {
        /* episode */
        env.physics_mut().set_time(0.0);
        let mut episode_reward = 0.0;
        let mut obs = env.reset();

        let mut step_count = 0;
        for _ in 0..episode_length {
            /* step */
            step_count += 1;
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
                TimeStep::Finish { observation, reward } => {
                    let next_state = env.task().state(&observation);
                    agent.learn(state, action, reward, next_state);
                    break; // End the episode
                }
            }
        }

        if episode % 1000 == 0 {
            println!(
                "[episode {episode:>7}]  return: {episode_reward:>10.2}  |  step: {step_count:>4}/{episode_length}  |  time: {:>5.2}[s]",
                env.physics().time(),
            );
        }
        if episode % model_save_interval == 0 {
            print!("----> saving current agent as a file...");
            std::io::Write::flush(&mut std::io::stdout()).unwrap();
            let filename = format!("agent_{episode}@{}.json", episode_reward.round() as usize);
            let path = model_save_directory.join(&filename);
            agent.save(&path).expect("Failed to save agent");
            println!(
                " 📄./{}",
                path.iter()
                    .skip(std::env::current_dir().unwrap().iter().count())
                    .collect::<std::path::PathBuf>()
                    .display()
            );
        }

        agent.decay_alpha_with_rate(0.9999);
        agent.decay_epsilon_with_rate(0.9999);
    }
}
