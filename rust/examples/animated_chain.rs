use glam::Vec3;
use ik_webgpu::collision::{CollisionConfig, CollisionHit, CollisionResponse, ObstacleWorld};
use ik_webgpu::dynamics::{SecondOrderDynamics, SpringPreset};
use ik_webgpu::ik::{Chain, FabrikSolver};
use ik_webgpu::render::{Camera, CameraController, DebugRenderer, GpuContext, Key, MouseAction};
use std::sync::Arc;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

struct App<'a> {
    window: Option<Arc<Window>>,
    context: Option<GpuContext<'a>>,
    renderer: Option<DebugRenderer>,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    chain: Chain,
    raw_target: Vec3,
    smoothed_target: Vec3,
    camera: Camera,
    controller: CameraController,
    target_dynamics: SecondOrderDynamics<Vec3>,
    joint_dynamics: Vec<SecondOrderDynamics<Vec3>>,
    smoothed_chain: Chain,
    obstacle_world: ObstacleWorld,
    collision_config: CollisionConfig,
    collision_enabled: bool,
    show_collision_hits: bool,
    collision_hits: Vec<CollisionHit>,
    window_size: (u32, u32),
    mouse_pos: (f32, f32),
    dragging_target: bool,
    dynamics_enabled: bool,
    last_frame: Instant,
    frequency: f32,
    damping: f32,
    response: f32,
    current_preset: usize,
    gui_hovered: bool,
}

const PRESET_NAMES: [&str; 5] = ["Smooth", "Snappy", "Bouncy", "Sluggish", "Anticipate"];

impl<'a> App<'a> {
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

        let mut obstacle_world = ObstacleWorld::new();
        obstacle_world.add_sphere(Vec3::new(1.5, 2.0, 0.0), 0.5);
        obstacle_world.add_sphere(Vec3::new(-1.0, 1.5, 0.0), 0.4);
        obstacle_world.add_box(Vec3::new(0.0, 3.0, 0.0), Vec3::new(0.8, 0.3, 0.3));

        let collision_config = CollisionConfig::new(0.1);

        let joint_positions: Vec<Vec3> = chain.joints().iter().map(|j| j.position).collect();
        let joint_dynamics: Vec<SecondOrderDynamics<Vec3>> = joint_positions
            .iter()
            .map(|&pos| SecondOrderDynamics::new(8.0, 1.0, 1.0, pos))
            .collect();

        let smoothed_chain = chain.clone();

        Self {
            window: None,
            context: None,
            renderer: None,
            egui_state: None,
            egui_renderer: None,
            chain,
            raw_target: initial_target,
            smoothed_target: initial_target,
            camera,
            controller,
            target_dynamics,
            joint_dynamics,
            smoothed_chain,
            obstacle_world,
            collision_config,
            collision_enabled: true,
            show_collision_hits: true,
            collision_hits: Vec::new(),
            window_size: (1280, 720),
            mouse_pos: (640.0, 360.0),
            dragging_target: false,
            dynamics_enabled: true,
            last_frame: Instant::now(),
            frequency: f,
            damping: z,
            response: r,
            current_preset: 0,
            gui_hovered: false,
        }
    }

    fn screen_to_ndc(&self, x: f32, y: f32) -> (f32, f32) {
        let (w, h) = self.window_size;
        let ndc_x = (2.0 * x / w as f32) - 1.0;
        let ndc_y = 1.0 - (2.0 * y / h as f32);
        (ndc_x, ndc_y)
    }

    fn apply_preset(&mut self, preset: SpringPreset) {
        let (f, z, r) = preset.params();
        self.frequency = f;
        self.damping = z;
        self.response = r;
        self.target_dynamics.set_parameters(f, z, r);
    }

    fn update(&mut self) {
        let now = Instant::now();
        let dt = (now - self.last_frame).as_secs_f32();
        self.last_frame = now;

        self.controller.update(&mut self.camera);

        let target = if self.dynamics_enabled {
            self.smoothed_target = self.target_dynamics.update(self.raw_target, dt);
            self.smoothed_target
        } else {
            self.raw_target
        };

        if self.collision_enabled {
            FabrikSolver::solve_with_collision(
                &mut self.chain,
                target,
                &self.obstacle_world,
                &self.collision_config,
            );

            self.collision_hits = CollisionResponse::get_collision_hits(
                &self.chain,
                &self.obstacle_world,
                self.collision_config.margin,
            );
        } else {
            FabrikSolver::solve(&mut self.chain, target);
            self.collision_hits.clear();
        }

        let chain_joints = self.chain.joints();
        let smoothed_joints = self.smoothed_chain.joints_mut();
        for (i, (dynamics, target_pos)) in self
            .joint_dynamics
            .iter_mut()
            .zip(chain_joints.iter().map(|j| j.position))
            .enumerate()
        {
            smoothed_joints[i].position = dynamics.update(target_pos, dt);
        }
    }

    fn render(&mut self) {
        if self.window.is_none() || self.context.is_none() || self.renderer.is_none()
            || self.egui_state.is_none() || self.egui_renderer.is_none() {
            return;
        }

        let window = self.window.as_ref().unwrap();
        let context = self.context.as_ref().unwrap();

        let output = match context.surface.get_current_texture() {
            Ok(output) => output,
            Err(_) => return,
        };

        let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

        let egui_state = self.egui_state.as_mut().unwrap();
        let raw_input = egui_state.take_egui_input(window);
        let egui_ctx = egui_state.egui_ctx().clone();

        let mut dynamics_enabled = self.dynamics_enabled;
        let mut collision_enabled = self.collision_enabled;
        let mut show_collision_hits = self.show_collision_hits;
        let mut current_preset = self.current_preset;
        let mut frequency = self.frequency;
        let mut damping = self.damping;
        let mut response = self.response;
        let mut preset_changed: Option<usize> = None;

        let full_output = egui_ctx.run(raw_input, |ctx| {
            egui::Window::new("IK Controls")
                .default_pos([10.0, 10.0])
                .resizable(false)
                .show(ctx, |ui| {
                    ui.checkbox(&mut dynamics_enabled, "Enable Dynamics");
                    ui.checkbox(&mut collision_enabled, "Enable Collision");
                    ui.checkbox(&mut show_collision_hits, "Show Collision Hits");
                    ui.separator();

                    ui.label("Presets:");
                    ui.horizontal(|ui| {
                        for (i, name) in PRESET_NAMES.iter().enumerate() {
                            if ui.selectable_label(current_preset == i, *name).clicked() {
                                current_preset = i;
                                preset_changed = Some(i);
                            }
                        }
                    });

                    ui.separator();
                    ui.label("Parameters:");

                    ui.horizontal(|ui| {
                        ui.label("Frequency:");
                        ui.add(egui::Slider::new(&mut frequency, 0.1..=10.0).suffix(" Hz"));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Damping:");
                        ui.add(egui::Slider::new(&mut damping, 0.0..=2.0));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Response:");
                        ui.add(egui::Slider::new(&mut response, -1.0..=3.0));
                    });

                    ui.separator();
                    ui.label("Damping Guide:");
                    ui.small("< 1.0: Bouncy (underdamped)");
                    ui.small("= 1.0: Smooth (critically damped)");
                    ui.small("> 1.0: Sluggish (overdamped)");

                    ui.separator();
                    ui.label("Response Guide:");
                    ui.small("< 0: Anticipation");
                    ui.small("= 0: Smooth start");
                    ui.small("= 1: Immediate");
                    ui.small("> 1: Overshoot");

                    ui.separator();
                    ui.label("Controls:");
                    ui.small("Left drag: Move target");
                    ui.small("Right drag: Orbit camera");
                    ui.small("Scroll: Zoom");
                    ui.small("WASD: Move camera");
                });
        });

        self.gui_hovered = egui_ctx.is_pointer_over_area();

        let egui_state = self.egui_state.as_mut().unwrap();
        egui_state.handle_platform_output(window, full_output.platform_output);
        let clipped_primitives = egui_ctx.tessellate(full_output.shapes, full_output.pixels_per_point);

        self.dynamics_enabled = dynamics_enabled;
        self.collision_enabled = collision_enabled;
        self.show_collision_hits = show_collision_hits;
        self.current_preset = current_preset;

        if let Some(i) = preset_changed {
            let preset = match i {
                0 => SpringPreset::Smooth,
                1 => SpringPreset::Snappy,
                2 => SpringPreset::Bouncy,
                3 => SpringPreset::Sluggish,
                4 => SpringPreset::Anticipate,
                _ => SpringPreset::Smooth,
            };
            self.apply_preset(preset);
        } else if frequency != self.frequency || damping != self.damping || response != self.response {
            self.frequency = frequency;
            self.damping = damping;
            self.response = response;
            self.target_dynamics.set_parameters(frequency, damping, response);
        }

        let context = self.context.as_ref().unwrap();
        let egui_renderer = self.egui_renderer.as_mut().unwrap();

        for (id, delta) in &full_output.textures_delta.set {
            egui_renderer.update_texture(&context.device, &context.queue, *id, delta);
        }

        let screen_descriptor = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [context.size.width, context.size.height],
            pixels_per_point: full_output.pixels_per_point,
        };

        let display_target = if self.dynamics_enabled {
            self.smoothed_target
        } else {
            self.raw_target
        };

        let renderer = self.renderer.as_ref().unwrap();
        let hits: &[CollisionHit] = if self.show_collision_hits {
            &self.collision_hits
        } else {
            &[]
        };
        renderer.render_with_collision_hits(
            context,
            &view,
            &self.smoothed_chain,
            display_target,
            &self.camera,
            &self.obstacle_world,
            hits,
        );

        self.render_egui(
            &view,
            clipped_primitives,
            screen_descriptor,
            full_output.textures_delta.free,
        );

        output.present();
    }

    fn render_egui(
        &mut self,
        view: &wgpu::TextureView,
        clipped_primitives: Vec<egui::ClippedPrimitive>,
        screen_descriptor: egui_wgpu::ScreenDescriptor,
        textures_to_free: Vec<egui::TextureId>,
    ) {
        let context = self.context.as_ref().unwrap();
        let mut egui_renderer = self.egui_renderer.take().unwrap();

        let mut encoder = context.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Egui Encoder"),
        });

        egui_renderer.update_buffers(
            &context.device,
            &context.queue,
            &mut encoder,
            &clipped_primitives,
            &screen_descriptor,
        );

        {
            let render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Egui Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            let mut render_pass = render_pass.forget_lifetime();
            egui_renderer.render(&mut render_pass, &clipped_primitives, &screen_descriptor);
        }

        context.queue.submit(std::iter::once(encoder.finish()));

        for id in &textures_to_free {
            egui_renderer.free_texture(id);
        }

        self.egui_renderer = Some(egui_renderer);
    }
}

impl ApplicationHandler for App<'_> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let window_attrs = Window::default_attributes()
                .with_title("FABRIK IK with Second Order Dynamics")
                .with_inner_size(winit::dpi::LogicalSize::new(1280, 720));

            let window = Arc::new(event_loop.create_window(window_attrs).unwrap());
            self.window = Some(window.clone());

            let context = pollster::block_on(GpuContext::new(window.clone()));
            self.window_size = (context.size.width, context.size.height);
            self.camera.set_aspect(context.aspect_ratio());

            let renderer = DebugRenderer::new(&context);

            let egui_ctx = egui::Context::default();
            let egui_state = egui_winit::State::new(
                egui_ctx,
                egui::ViewportId::ROOT,
                &window,
                Some(window.scale_factor() as f32),
                None,
                None,
            );

            let egui_renderer = egui_wgpu::Renderer::new(
                &context.device,
                context.config.format,
                None,
                1,
                false,
            );

            self.context = Some(context);
            self.renderer = Some(renderer);
            self.egui_state = Some(egui_state);
            self.egui_renderer = Some(egui_renderer);
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        if let Some(egui_state) = &mut self.egui_state {
            if let Some(window) = &self.window {
                let response = egui_state.on_window_event(window, &event);
                if response.consumed {
                    return;
                }
            }
        }

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),

            WindowEvent::Resized(size) => {
                if let Some(context) = &mut self.context {
                    context.resize(size);
                    self.window_size = (size.width, size.height);
                    self.camera.set_aspect(context.aspect_ratio());
                }
            }

            WindowEvent::KeyboardInput { event, .. } => {
                let pressed = event.state == ElementState::Pressed;
                if let PhysicalKey::Code(code) = event.physical_key {
                    match code {
                        KeyCode::Escape => event_loop.exit(),
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
                if !self.gui_hovered {
                    let pressed = state == ElementState::Pressed;
                    match button {
                        MouseButton::Left => self.dragging_target = pressed,
                        MouseButton::Right => self.controller.on_mouse_button(1, pressed),
                        MouseButton::Middle => self.controller.on_mouse_button(2, pressed),
                        _ => {}
                    }
                }
            }

            WindowEvent::CursorMoved { position, .. } => {
                let (x, y) = (position.x as f32, position.y as f32);

                if self.dragging_target && !self.gui_hovered {
                    let (ndc_x, ndc_y) = self.screen_to_ndc(x, y);
                    if let Some(world_pos) = self.camera.screen_to_world_on_plane(ndc_x, ndc_y, Vec3::Z, 0.0) {
                        self.raw_target = world_pos;
                    }
                }

                self.controller.on_mouse_move(x, y);
                self.mouse_pos = (x, y);
            }

            WindowEvent::MouseWheel { delta, .. } => {
                if !self.gui_hovered {
                    let scroll = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(pos) => pos.y as f32 / 50.0,
                    };
                    self.controller.on_scroll(scroll);
                }
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
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();

    let event_loop = EventLoop::new().unwrap();
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App::new();
    event_loop.run_app(&mut app).unwrap();
}
