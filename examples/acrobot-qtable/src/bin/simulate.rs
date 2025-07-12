use acrobot_qtable::*;
use qtable::strategy;
use oxide_control::TimeStep;
use oxide_control::physics::binding::{
    mjr_render, mjr_makeContext, mjrContext, mjrRect,
    mjv_updateScene, mjv_makeScene, mjvCamera, mjvOption, mjvScene,
    mjtCatBit, mjtFontScale,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let target_qtable_path = args.next().expect("Usage: simulate <target_qtable_path>");

    let t = TrainedAgent::load(&target_qtable_path)
        .expect(&format!("Failed to load trained agent from file `{target_qtable_path}`"));

    let mut env = oxide_control::Environment::new::<AcrobotAction>(
        Acrobot::new(),
        AcrobotBalanceTask {
            do_swing: false,
            discount: 0.99,
            n_arm_digitization: t.n_arm_digitization(),
            n_pendulum_digitization: t.n_pendulum_digitization(),
        },
    );

    let mut cam = mjvCamera::default();
    let opt = mjvOption::default();
    let mut scn = mjvScene::default();
    let mut con = mjrContext::default();
    mjv_makeScene(&env.physics().model().binding, &mut scn, 2000);
    mjr_makeContext(&env.physics().model().binding, &mut con, mjtFontScale::X150);

    let mut glfw = glfw::init(glfw::fail_on_errors).expect("Failed to initialize GLFW");
    let (mut window, events) = glfw.create_window(
        1200, 900, "Acrobot Simulation", glfw::WindowMode::Windowed
    ).expect("Failed to create GLFW window");
    window.set_size_polling(true);
    glfw::Context::make_current(&mut *window);

    let mut obs = env.reset();
    while !window.should_close() {
        while env.physics().time() < glfw.get_time() {            
            match env.step(t.get_action::<strategy::MostQValue>(env.task().state(&obs))) {
                TimeStep::Step { observation, .. } => {
                    obs = observation;
                }
                TimeStep::Finish { observation, .. } => {
                    obs = observation;
                    /* Not `window.set_should_close(true);` to keep rendering */
                }
            }
        }

        let mut viewport = mjrRect::new(0, 0, 0, 0);
        let (width, height) = window.get_framebuffer_size();
        viewport.set_width(width);
        viewport.set_height(height);

        let (model, data) = env.physics_mut().model_datamut();
        mjv_updateScene(
            &model.binding,
            &mut data.binding,
            &opt,
            None, // No perturbation
            &mut cam,
            mjtCatBit::ALL,
            &mut scn,
        );
        mjr_render(
            viewport,
            &mut scn,
            &con,
        );
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
                _ => ()
            }
        }
    }
}
