//! WASM entry point - animated IK chain with second-order dynamics

use crate::dynamics::{SecondOrderDynamics, SpringPreset};
use crate::ik::{Chain, FabrikSolver};
use crate::render::{Camera, CameraController, DebugRenderer, GpuContext, Key, MouseAction};
use glam::Vec3;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use wasm_bindgen::prelude::*;
use web_sys::HtmlCanvasElement;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::platform::web::WindowAttributesExtWebSys;
use winit::platform::web::EventLoopExtWebSys;
use winit::window::{Window, WindowId};

struct AppState {
    context: Option<GpuContext<'static>>,
    renderer: Option<DebugRenderer>,
}

struct App {
    window: Option<Arc<Window>>,
    state: Rc<RefCell<AppState>>,
    chain: Chain,
    raw_target: Vec3,
    smoothed_target: Vec3,
    camera: Camera,
    controller: CameraController,
    target_dynamics: SecondOrderDynamics<Vec3>,
    window_size: (u32, u32),
    mouse_pos: (f32, f32),
    dragging_target: bool,
    last_time: f64,
    init_pending: bool,
}

impl App {
    fn new() -> Self {
        let chain = Chain::builder()
            .add_joint(Vec3::ZERO)
            .add_joint(Vec3::new(0.0, 0.8, 0.0))
            .add_joint(Vec3::new(0.0, 1.6, 0.0))
            .add_joint(Vec3::new(0.0, 2.4, 0.0))
            .add_joint(Vec3::new(0.0, 3.2, 0.0))
            .add_joint(Vec3::new(0.0, 4.0, 0.0))
            .tolerance(0.001)
            .max_iterations(20)
            .build();

        let initial_target = Vec3::new(2.0, 2.0, 0.0);
        let camera = Camera::default();
        let mut controller = CameraController::new(Vec3::new(0.0, 2.0, 0.0), 10.0);
        controller.orbit.phi = 1.2;
        controller.left_mouse_action = MouseAction::None;
        controller.right_mouse_action = MouseAction::Orbit;
        controller.middle_mouse_action = MouseAction::Pan;

        let (f, z, r) = SpringPreset::Smooth.params();
        let target_dynamics = SecondOrderDynamics::new(f, z, r, initial_target);

        Self {
            window: None,
            state: Rc::new(RefCell::new(AppState {
                context: None,
                renderer: None,
            })),
            chain,
            raw_target: initial_target,
            smoothed_target: initial_target,
            camera,
            controller,
            target_dynamics,
            window_size: (1280, 720),
            mouse_pos: (640.0, 360.0),
            dragging_target: false,
            last_time: 0.0,
            init_pending: false,
        }
    }

    fn screen_to_ndc(&self, x: f32, y: f32) -> (f32, f32) {
        let (w, h) = self.window_size;
        let ndc_x = (2.0 * x / w as f32) - 1.0;
        let ndc_y = 1.0 - (2.0 * y / h as f32);
        (ndc_x, ndc_y)
    }

    fn update(&mut self, dt: f32) {
        self.controller.update(&mut self.camera);
        self.smoothed_target = self.target_dynamics.update(self.raw_target, dt);
        FabrikSolver::solve(&mut self.chain, self.smoothed_target);
    }

    fn render(&mut self) {
        let state = self.state.borrow();
        let context = match state.context.as_ref() {
            Some(c) => c,
            None => return,
        };
        let renderer = match state.renderer.as_ref() {
            Some(r) => r,
            None => return,
        };

        let output = match context.surface.get_current_texture() {
            Ok(o) => o,
            Err(_) => return,
        };

        let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());
        renderer.render(context, &view, &self.chain, self.smoothed_target, &self.camera);
        output.present();
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() || self.init_pending {
            return;
        }
        self.init_pending = true;

        let canvas = web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id("canvas"))
            .and_then(|e| e.dyn_into::<HtmlCanvasElement>().ok())
            .expect("Could not find canvas element with id 'canvas'");

        let width = canvas.width().max(1);
        let height = canvas.height().max(1);

        let window_attrs = Window::default_attributes()
            .with_canvas(Some(canvas))
            .with_inner_size(PhysicalSize::new(width, height));

        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());
        self.window = Some(window.clone());
        self.window_size = (width, height);

        let state = self.state.clone();

        wasm_bindgen_futures::spawn_local(async move {
            let context = GpuContext::new(window.clone()).await;
            // Transmute to 'static - safe because WASM is single-threaded and
            // we control the lifetime
            let context: GpuContext<'static> = unsafe { std::mem::transmute(context) };
            let renderer = DebugRenderer::new(&context);

            let mut state = state.borrow_mut();
            state.context = Some(context);
            state.renderer = Some(renderer);

            window.request_redraw();
        });
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),

            WindowEvent::Resized(size) => {
                let mut state = self.state.borrow_mut();
                if let Some(context) = &mut state.context {
                    context.resize(size);
                    self.window_size = (size.width, size.height);
                    self.camera.set_aspect(context.aspect_ratio());
                }
            }

            WindowEvent::KeyboardInput { event, .. } => {
                let pressed = event.state == ElementState::Pressed;
                if let PhysicalKey::Code(code) = event.physical_key {
                    match code {
                        KeyCode::KeyW => self.controller.on_key(Key::W, pressed),
                        KeyCode::KeyS => self.controller.on_key(Key::S, pressed),
                        KeyCode::KeyA => self.controller.on_key(Key::A, pressed),
                        KeyCode::KeyD => self.controller.on_key(Key::D, pressed),
                        KeyCode::KeyQ => self.controller.on_key(Key::Q, pressed),
                        KeyCode::KeyE => self.controller.on_key(Key::E, pressed),
                        KeyCode::KeyR if pressed => self.controller.reset(),
                        _ => {}
                    }
                }
            }

            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = state == ElementState::Pressed;
                match button {
                    MouseButton::Left => self.dragging_target = pressed,
                    MouseButton::Right => self.controller.on_mouse_button(1, pressed),
                    MouseButton::Middle => self.controller.on_mouse_button(2, pressed),
                    _ => {}
                }
            }

            WindowEvent::CursorMoved { position, .. } => {
                let (x, y) = (position.x as f32, position.y as f32);

                if self.dragging_target {
                    let (ndc_x, ndc_y) = self.screen_to_ndc(x, y);
                    if let Some(world_pos) = self.camera.screen_to_world_on_plane(ndc_x, ndc_y, Vec3::Z, 0.0) {
                        self.raw_target = world_pos;
                    }
                }

                self.controller.on_mouse_move(x, y);
                self.mouse_pos = (x, y);
            }

            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(pos) => pos.y as f32 / 50.0,
                };
                self.controller.on_scroll(scroll);
            }

            WindowEvent::RedrawRequested => {
                // Update camera aspect from state
                {
                    let state = self.state.borrow();
                    if let Some(context) = &state.context {
                        self.camera.set_aspect(context.aspect_ratio());
                    }
                }

                let now = web_sys::window()
                    .and_then(|w| w.performance())
                    .map(|p| p.now())
                    .unwrap_or(0.0);

                let dt = if self.last_time > 0.0 {
                    ((now - self.last_time) / 1000.0) as f32
                } else {
                    1.0 / 60.0
                };
                self.last_time = now;

                self.update(dt.min(0.1));
                self.render();

                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }

            _ => {}
        }
    }
}

#[wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
    console_log::init_with_level(log::Level::Warn).expect("Failed to init logger");

    let event_loop = EventLoop::new().unwrap();
    let app = App::new();

    event_loop.spawn_app(app);
}