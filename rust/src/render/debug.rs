use super::camera::Camera;
use super::context::GpuContext;
use super::mesh::Mesh;

use super::pipeline::{RenderPipelines, Uniforms};
use crate::collision::{CollisionHit, ObstacleShape, ObstacleWorld};
use crate::ik::Chain;
use glam::{Mat4, Vec3};

const MAX_INSTANCES: usize = 64;

pub struct DebugRenderer {
    pipelines: RenderPipelines,
    line_pipeline: wgpu::RenderPipeline,
    sphere_mesh: Mesh,
    cylinder_mesh: Mesh,
    wireframe_sphere_mesh: Mesh,
    wireframe_box_mesh: Mesh,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    uniform_alignment: u32,
}

impl DebugRenderer {
    pub fn new(context: &GpuContext) -> Self {
        let pipelines = RenderPipelines::new(context);
        let line_pipeline = pipelines.create_line_pipeline(context);
        let sphere_mesh = Mesh::sphere(&context.device, 1.0, 16, 12);
        let cylinder_mesh = Mesh::cylinder(&context.device, 1.0, 1.0, 12);
        let wireframe_sphere_mesh = Mesh::wireframe_sphere(&context.device, 1.0, 16, 8);
        let wireframe_box_mesh = Mesh::wireframe_box(&context.device, Vec3::ONE);

        let uniform_alignment = context.device.limits().min_uniform_buffer_offset_alignment;
        let aligned_size = Self::align_to(std::mem::size_of::<Uniforms>() as u32, uniform_alignment);
        let buffer_size = (aligned_size as usize * MAX_INSTANCES) as u64;

        let uniform_buffer = context.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Dynamic Uniform Buffer"),
            size: buffer_size,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group = pipelines.create_dynamic_bind_group(&context.device, &uniform_buffer);

        Self {
            pipelines,
            line_pipeline,
            sphere_mesh,
            cylinder_mesh,
            wireframe_sphere_mesh,
            wireframe_box_mesh,
            uniform_buffer,
            bind_group,
            uniform_alignment,
        }
    }

    fn align_to(size: u32, alignment: u32) -> u32 {
        (size + alignment - 1) & !(alignment - 1)
    }

    fn aligned_uniform_size(&self) -> u32 {
        Self::align_to(std::mem::size_of::<Uniforms>() as u32, self.uniform_alignment)
    }

    pub fn render(
        &self,
        context: &GpuContext,
        view: &wgpu::TextureView,
        chain: &Chain,
        target: Vec3,
        camera: &Camera,
    ) {
        let view_proj = camera.view_projection();
        let aligned_size = self.aligned_uniform_size() as usize;

        let mut uniform_data = vec![0u8; aligned_size * MAX_INSTANCES];
        let mut instance_idx = 0;

        struct DrawCall {
            is_sphere: bool,
            offset: u32,
        }
        let mut draw_calls: Vec<DrawCall> = Vec::new();

        let joints = chain.joints();
        for (i, joint) in joints.iter().enumerate() {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let is_root = i == 0;
            let is_end = i == joints.len() - 1;
            let color = if is_root {
                [0.2, 0.8, 0.2, 1.0]
            } else if is_end {
                [0.8, 0.8, 0.2, 1.0]
            } else {
                [0.3, 0.5, 0.9, 1.0]
            };

            let model = Mat4::from_translation(joint.position) * Mat4::from_scale(Vec3::splat(0.08));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color,
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            draw_calls.push(DrawCall {
                is_sphere: true,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        for i in 0..joints.len().saturating_sub(1) {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let start = joints[i].position;
            let end = joints[i + 1].position;
            let model = Mesh::create_bone_transform(start, end) * Mat4::from_scale(Vec3::new(0.03, 1.0, 0.03));

            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [0.6, 0.6, 0.7, 1.0],
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            draw_calls.push(DrawCall {
                is_sphere: false,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        if instance_idx < MAX_INSTANCES {
            let model = Mat4::from_translation(target) * Mat4::from_scale(Vec3::splat(0.12));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [1.0, 0.2, 0.2, 1.0],
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            draw_calls.push(DrawCall {
                is_sphere: true,
                offset: offset as u32,
            });
        }

        context.queue.write_buffer(&self.uniform_buffer, 0, &uniform_data);

        let mut encoder = context.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.15,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &context.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            render_pass.set_pipeline(&self.pipelines.pipeline);

            for call in &draw_calls {
                render_pass.set_bind_group(0, &self.bind_group, &[call.offset]);

                if call.is_sphere {
                    render_pass.set_vertex_buffer(0, self.sphere_mesh.vertex_buffer.slice(..));
                    render_pass.set_index_buffer(self.sphere_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    render_pass.draw_indexed(0..self.sphere_mesh.index_count, 0, 0..1);
                } else {
                    render_pass.set_vertex_buffer(0, self.cylinder_mesh.vertex_buffer.slice(..));
                    render_pass.set_index_buffer(self.cylinder_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    render_pass.draw_indexed(0..self.cylinder_mesh.index_count, 0, 0..1);
                }
            }
        }

        context.queue.submit(std::iter::once(encoder.finish()));
    }

    pub fn render_with_obstacles(
        &self,
        context: &GpuContext,
        view: &wgpu::TextureView,
        chain: &Chain,
        target: Vec3,
        camera: &Camera,
        world: &ObstacleWorld,
    ) {
        let view_proj = camera.view_projection();
        let aligned_size = self.aligned_uniform_size() as usize;

        let mut uniform_data = vec![0u8; aligned_size * MAX_INSTANCES];
        let mut instance_idx = 0;

        #[derive(Clone, Copy)]
        enum MeshType {
            Sphere,
            Cylinder,
            WireframeSphere,
            WireframeBox,
        }

        struct DrawCall {
            mesh_type: MeshType,
            offset: u32,
        }
        let mut solid_draw_calls: Vec<DrawCall> = Vec::new();
        let mut wireframe_draw_calls: Vec<DrawCall> = Vec::new();

        let joints = chain.joints();
        for (i, joint) in joints.iter().enumerate() {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let is_root = i == 0;
            let is_end = i == joints.len() - 1;
            let color = if is_root {
                [0.2, 0.8, 0.2, 1.0]
            } else if is_end {
                [0.8, 0.8, 0.2, 1.0]
            } else {
                [0.3, 0.5, 0.9, 1.0]
            };

            let model = Mat4::from_translation(joint.position) * Mat4::from_scale(Vec3::splat(0.08));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color,
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Sphere,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        for i in 0..joints.len().saturating_sub(1) {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let start = joints[i].position;
            let end = joints[i + 1].position;
            let model = Mesh::create_bone_transform(start, end) * Mat4::from_scale(Vec3::new(0.03, 1.0, 0.03));

            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [0.6, 0.6, 0.7, 1.0],
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Cylinder,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        if instance_idx < MAX_INSTANCES {
            let model = Mat4::from_translation(target) * Mat4::from_scale(Vec3::splat(0.12));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [1.0, 0.2, 0.2, 1.0],
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Sphere,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        for obstacle in world.obstacles() {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let (model, mesh_type, color) = match obstacle.render_shape() {
                ObstacleShape::Sphere { center, radius } => (
                    Mat4::from_translation(center) * Mat4::from_scale(Vec3::splat(radius)),
                    MeshType::WireframeSphere,
                    [1.0, 0.5, 0.0, 1.0], 
                ),
                ObstacleShape::Box { center, half_extents } => (
                    Mat4::from_translation(center) * Mat4::from_scale(half_extents),
                    MeshType::WireframeBox,
                    [0.0, 1.0, 0.5, 1.0], 
                ),
            };

            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color,
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            wireframe_draw_calls.push(DrawCall {
                mesh_type,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        context.queue.write_buffer(&self.uniform_buffer, 0, &uniform_data);

        let mut encoder = context.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.15,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &context.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            render_pass.set_pipeline(&self.pipelines.pipeline);
            for call in &solid_draw_calls {
                render_pass.set_bind_group(0, &self.bind_group, &[call.offset]);
                match call.mesh_type {
                    MeshType::Sphere => {
                        render_pass.set_vertex_buffer(0, self.sphere_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.sphere_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.sphere_mesh.index_count, 0, 0..1);
                    }
                    MeshType::Cylinder => {
                        render_pass.set_vertex_buffer(0, self.cylinder_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.cylinder_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.cylinder_mesh.index_count, 0, 0..1);
                    }
                    _ => {}
                }
            }

            render_pass.set_pipeline(&self.line_pipeline);
            for call in &wireframe_draw_calls {
                render_pass.set_bind_group(0, &self.bind_group, &[call.offset]);
                match call.mesh_type {
                    MeshType::WireframeSphere => {
                        render_pass.set_vertex_buffer(0, self.wireframe_sphere_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.wireframe_sphere_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.wireframe_sphere_mesh.index_count, 0, 0..1);
                    }
                    MeshType::WireframeBox => {
                        render_pass.set_vertex_buffer(0, self.wireframe_box_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.wireframe_box_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.wireframe_box_mesh.index_count, 0, 0..1);
                    }
                    _ => {}
                }
            }
        }

        context.queue.submit(std::iter::once(encoder.finish()));
    }

    pub fn render_with_collision_hits(
        &self,
        context: &GpuContext,
        view: &wgpu::TextureView,
        chain: &Chain,
        target: Vec3,
        camera: &Camera,
        world: &ObstacleWorld,
        collision_hits: &[CollisionHit],
    ) {
        let view_proj = camera.view_projection();
        let aligned_size = self.aligned_uniform_size() as usize;

        let mut uniform_data = vec![0u8; aligned_size * MAX_INSTANCES];
        let mut instance_idx = 0;

        #[derive(Clone, Copy)]
        enum MeshType {
            Sphere,
            Cylinder,
            WireframeSphere,
            WireframeBox,
        }

        struct DrawCall {
            mesh_type: MeshType,
            offset: u32,
        }
        let mut solid_draw_calls: Vec<DrawCall> = Vec::new();
        let mut wireframe_draw_calls: Vec<DrawCall> = Vec::new();

        let joints = chain.joints();
        for (i, joint) in joints.iter().enumerate() {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let is_root = i == 0;
            let is_end = i == joints.len() - 1;
            let color = if is_root {
                [0.2, 0.8, 0.2, 1.0]
            } else if is_end {
                [0.8, 0.8, 0.2, 1.0]
            } else {
                [0.3, 0.5, 0.9, 1.0]
            };

            let model = Mat4::from_translation(joint.position) * Mat4::from_scale(Vec3::splat(0.08));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color,
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Sphere,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        for i in 0..joints.len().saturating_sub(1) {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let start = joints[i].position;
            let end = joints[i + 1].position;
            let model = Mesh::create_bone_transform(start, end) * Mat4::from_scale(Vec3::new(0.03, 1.0, 0.03));

            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [0.6, 0.6, 0.7, 1.0],
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Cylinder,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        if instance_idx < MAX_INSTANCES {
            let model = Mat4::from_translation(target) * Mat4::from_scale(Vec3::splat(0.12));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [1.0, 0.2, 0.2, 1.0],
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Sphere,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        for hit in collision_hits {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let model = Mat4::from_translation(hit.surface_point) * Mat4::from_scale(Vec3::splat(0.06));
            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color: [1.0, 0.0, 1.0, 1.0], 
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            solid_draw_calls.push(DrawCall {
                mesh_type: MeshType::Sphere,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        for obstacle in world.obstacles() {
            if instance_idx >= MAX_INSTANCES {
                break;
            }

            let (model, mesh_type, color) = match obstacle.render_shape() {
                ObstacleShape::Sphere { center, radius } => (
                    Mat4::from_translation(center) * Mat4::from_scale(Vec3::splat(radius)),
                    MeshType::WireframeSphere,
                    [1.0, 0.5, 0.0, 1.0],  
                ),
                ObstacleShape::Box { center, half_extents } => (
                    Mat4::from_translation(center) * Mat4::from_scale(half_extents),
                    MeshType::WireframeBox,
                    [0.0, 1.0, 0.5, 1.0], 
                ),
            };

            let uniforms = Uniforms {
                view_proj: view_proj.to_cols_array_2d(),
                model: model.to_cols_array_2d(),
                color,
            };

            let offset = instance_idx * aligned_size;
            let bytes = bytemuck::bytes_of(&uniforms);
            uniform_data[offset..offset + bytes.len()].copy_from_slice(bytes);

            wireframe_draw_calls.push(DrawCall {
                mesh_type,
                offset: offset as u32,
            });
            instance_idx += 1;
        }

        context.queue.write_buffer(&self.uniform_buffer, 0, &uniform_data);

        let mut encoder = context.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.15,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &context.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            render_pass.set_pipeline(&self.pipelines.pipeline);
            for call in &solid_draw_calls {
                render_pass.set_bind_group(0, &self.bind_group, &[call.offset]);
                match call.mesh_type {
                    MeshType::Sphere => {
                        render_pass.set_vertex_buffer(0, self.sphere_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.sphere_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.sphere_mesh.index_count, 0, 0..1);
                    }
                    MeshType::Cylinder => {
                        render_pass.set_vertex_buffer(0, self.cylinder_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.cylinder_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.cylinder_mesh.index_count, 0, 0..1);
                    }
                    _ => {}
                }
            }

            render_pass.set_pipeline(&self.line_pipeline);
            for call in &wireframe_draw_calls {
                render_pass.set_bind_group(0, &self.bind_group, &[call.offset]);
                match call.mesh_type {
                    MeshType::WireframeSphere => {
                        render_pass.set_vertex_buffer(0, self.wireframe_sphere_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.wireframe_sphere_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.wireframe_sphere_mesh.index_count, 0, 0..1);
                    }
                    MeshType::WireframeBox => {
                        render_pass.set_vertex_buffer(0, self.wireframe_box_mesh.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.wireframe_box_mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(0..self.wireframe_box_mesh.index_count, 0, 0..1);
                    }
                    _ => {}
                }
            }
        }

        context.queue.submit(std::iter::once(encoder.finish()));
    }
}