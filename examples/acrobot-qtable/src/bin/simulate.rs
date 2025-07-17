use acrobot_qtable::*;
use oxide_control::TimeStep;
use oxide_control::physics::binding::{
    mjr_makeContext, mjr_render, mjrContext, mjrRect, mjtCatBit, mjtFontScale, mjv_makeScene, mjv_updateScene, mjvCamera, mjvOption, mjvScene,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let target_agent_path = args.next().expect("Usage: simulate <target_agent_path>");

    println!("Loading trained agent from `{target_agent_path}`...");

    let t = TrainedAgent::load(&target_agent_path).expect(&format!("Failed to load trained agent from file `{target_agent_path}`"));

    println!("Starting simulation with the agent...");

    let mut env = oxide_control::Environment::new(
        Acrobot::new(),
        AcrobotBalanceTask {
            action_size: t.action_size(),
            n_arm_digitization: t.n_arm_digitization(),
            n_pendulum_digitization: t.n_pendulum_digitization(),
            get_reward: |_, _, _| 0.0, /* No reward function needed for simulation */
        },
    );

    let mut glfw = glfw::init(glfw::fail_on_errors).expect("Failed to initialize GLFW");
    let (mut window, events) = glfw
        .create_window(1200, 900, "Acrobot Simulation", glfw::WindowMode::Windowed)
        .expect("Failed to create GLFW window");
    window.set_size_polling(true);
    glfw::Context::make_current(&mut *window);

    let opt = mjvOption::default();
    let mut scn = mjvScene::default();
    let mut con = mjrContext::default();
    let mut cam = mjvCamera::default();
    cam.set_fixedcamid(env.physics().object_id_of("fixed").unwrap());
    mjv_makeScene(&env.physics().model().binding, &mut scn, 2000);
    mjr_makeContext(&env.physics().model().binding, &mut con, mjtFontScale::X150);

    let mut obs = env.reset();
    while !window.should_close() {
        while env.physics().time() < glfw.get_time() {
            match env.step(t.get_action(env.task().state(&obs))) {
                TimeStep::Step { observation, .. } => {
                    obs = observation;
                }
                TimeStep::Finish { .. } => {
                    window.set_title("Acrobot Simulation - FINISHED");
                    std::thread::sleep(std::time::Duration::from_secs(3));
                    window.set_should_close(true);
                    break;
                }
            }
        }

        let viewport = {
            let (width, height) = window.get_framebuffer_size();
            mjrRect::new(0, 0, width as u32, height as u32)
        };

        let (model, data) = env.physics_mut().model_datamut();
        mjv_updateScene(
            &model.binding,
            &mut data.binding,
            &opt,
            None, /* No perturbation */
            &mut cam,
            mjtCatBit::ALL,
            &mut scn,
        );
        mjr_render(viewport, &mut scn, &con);
        glfw::Context::swap_buffers(&mut *window);

        glfw.poll_events();
        for (_, event) in glfw::flush_messages(&events) {
            match event {
                glfw::WindowEvent::Close => {
                    window.set_should_close(true);
                }
                glfw::WindowEvent::Size(width, height) => {
                    window.set_size(width, height);
                }
                _ => (),
            }
        }
    }
}
