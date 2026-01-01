use glam::Vec3;
use ik_webgpu::ik::{BallSocketConstraint, Chain, FabrikSolver};
use ik_webgpu::render::{Camera, DebugRenderer, GpuContext, OrbitController};
use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalPosition;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

struct App<'a> {
    window: Option<Arc<Window>>,
    context: Option<GpuContext<'a>>,
    renderer: Option<DebugRenderer>,
    chain: Chain,
    target: Vec3,
    camera: Camera,
    orbit: OrbitController,
    mouse_pos: PhysicalPosition<f64>,
    mouse_pressed: bool,
    right_mouse_pressed: bool,
}

impl<'a> App<'a> {
    fn new() -> Self {
        let chain = Chain::builder()
            .add_joint(Vec3::ZERO)
            .add_joint_with_constraint(Vec3::new(0.0, 1.0, 0.0), BallSocketConstraint::new(60.0))
            .add_joint_with_constraint(Vec3::new(0.0, 2.0, 0.0), BallSocketConstraint::new(60.0))
            .add_joint_with_constraint(Vec3::new(0.0, 3.0, 0.0), BallSocketConstraint::new(60.0))
            .add_joint(Vec3::new(0.0, 4.0, 0.0))
            .tolerance(0.001)
            .max_iterations(20)
            .build();

        let target = Vec3::new(2.0, 2.0, 0.0);
        let camera = Camera::default();
        let orbit = OrbitController::new(Vec3::new(0.0, 2.0, 0.0), 8.0);

        Self {
            window: None,
            context: None,
            renderer: None,
            chain,
            target,
            camera,
            orbit,
            mouse_pos: PhysicalPosition::new(0.0, 0.0),
            mouse_pressed: false,
            right_mouse_pressed: false,
        }
    }

    fn update(&mut self) {
        FabrikSolver::solve(&mut self.chain, self.target);
    }

    fn render(&mut self) {
        let context = self.context.as_ref().unwrap();
        let renderer = self.renderer.as_ref().unwrap();

        let output = match context.surface.get_current_texture() {
            Ok(output) => output,
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                return;
            }
            Err(e) => {
                log::error!("Surface error: {:?}", e);
                return;
            }
        };

        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        renderer.render(context, &view, &self.chain, self.target, &self.camera);

        output.present();
    }
}

impl ApplicationHandler for App<'_> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let window_attrs = Window::default_attributes()
                .with_title("FABRIK IK Demo")
                .with_inner_size(winit::dpi::LogicalSize::new(1280, 720));

            let window = Arc::new(event_loop.create_window(window_attrs).unwrap());
            self.window = Some(window.clone());

            let context = pollster::block_on(GpuContext::new(window));
            self.camera.set_aspect(context.aspect_ratio());
            self.orbit.update_camera(&mut self.camera);

            let renderer = DebugRenderer::new(&context);

            self.context = Some(context);
            self.renderer = Some(renderer);
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == ElementState::Pressed {
                    if let PhysicalKey::Code(code) = event.physical_key {
                        match code {
                            KeyCode::Escape => event_loop.exit(),
                            KeyCode::ArrowUp => self.target.y += 0.1,
                            KeyCode::ArrowDown => self.target.y -= 0.1,
                            KeyCode::ArrowLeft => self.target.x -= 0.1,
                            KeyCode::ArrowRight => self.target.x += 0.1,
                            KeyCode::KeyW => self.target.z -= 0.1,
                            KeyCode::KeyS => self.target.z += 0.1,
                            KeyCode::KeyR => {
                                self.target = Vec3::new(2.0, 2.0, 0.0);
                            }
                            _ => {}
                        }
                    }
                }
            }
            WindowEvent::Resized(size) => {
                if let Some(context) = &mut self.context {
                    context.resize(size);
                    self.camera.set_aspect(context.aspect_ratio());
                }
            }
            WindowEvent::MouseInput { state, button, .. } => match button {
                MouseButton::Left => {
                    self.mouse_pressed = state == ElementState::Pressed;
                }
                MouseButton::Right => {
                    self.right_mouse_pressed = state == ElementState::Pressed;
                }
                _ => {}
            },
            WindowEvent::CursorMoved { position, .. } => {
                let delta_x = position.x - self.mouse_pos.x;
                let delta_y = position.y - self.mouse_pos.y;
                self.mouse_pos = position;

                if self.mouse_pressed {
                    self.orbit.rotate(delta_x as f32, delta_y as f32);
                    self.orbit.update_camera(&mut self.camera);
                } else if self.right_mouse_pressed {
                    self.orbit.pan(-delta_x as f32, delta_y as f32);
                    self.orbit.update_camera(&mut self.camera);
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(pos) => pos.y as f32 / 50.0,
                };
                self.orbit.zoom(scroll);
                self.orbit.update_camera(&mut self.camera);
            }
            WindowEvent::RedrawRequested => {
                self.update();
                self.render();
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            _ => {}
        }
    }
}

fn main() {
    env_logger::init();

    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App::new();
    event_loop.run_app(&mut app).unwrap();
}