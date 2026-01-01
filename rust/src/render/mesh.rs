use bytemuck::{Pod, Zeroable};
use glam::Vec3;
use std::f32::consts::PI;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

impl Vertex {
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            },
            wgpu::VertexAttribute {
                offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                shader_location: 1,
                format: wgpu::VertexFormat::Float32x3,
            },
        ],
    };
}

pub struct Mesh {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
}

impl Mesh {
    pub fn from_data(device: &wgpu::Device, vertices: &[Vertex], indices: &[u32]) -> Self {
        use wgpu::util::DeviceExt;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Index Buffer"),
            contents: bytemuck::cast_slice(indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Self {
            vertex_buffer,
            index_buffer,
            index_count: indices.len() as u32,
        }
    }

    pub fn sphere(device: &wgpu::Device, radius: f32, segments: u32, rings: u32) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for ring in 0..=rings {
            let phi = PI * ring as f32 / rings as f32;
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();

            for seg in 0..=segments {
                let theta = 2.0 * PI * seg as f32 / segments as f32;
                let sin_theta = theta.sin();
                let cos_theta = theta.cos();

                let x = sin_phi * cos_theta;
                let y = cos_phi;
                let z = sin_phi * sin_theta;

                vertices.push(Vertex {
                    position: [x * radius, y * radius, z * radius],
                    normal: [x, y, z],
                });
            }
        }

        for ring in 0..rings {
            for seg in 0..segments {
                let curr_ring = ring * (segments + 1);
                let next_ring = (ring + 1) * (segments + 1);

                indices.push(curr_ring + seg);
                indices.push(next_ring + seg);
                indices.push(next_ring + seg + 1);

                indices.push(curr_ring + seg);
                indices.push(next_ring + seg + 1);
                indices.push(curr_ring + seg + 1);
            }
        }

        Self::from_data(device, &vertices, &indices)
    }

    pub fn cylinder(device: &wgpu::Device, radius: f32, height: f32, segments: u32) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let half_height = height / 2.0;

        for i in 0..=segments {
            let theta = 2.0 * PI * i as f32 / segments as f32;
            let x = theta.cos();
            let z = theta.sin();

            vertices.push(Vertex {
                position: [x * radius, -half_height, z * radius],
                normal: [x, 0.0, z],
            });

            vertices.push(Vertex {
                position: [x * radius, half_height, z * radius],
                normal: [x, 0.0, z],
            });
        }

        for i in 0..segments {
            let base = i * 2;
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 3);

            indices.push(base);
            indices.push(base + 3);
            indices.push(base + 2);
        }

        let base_center_idx = vertices.len() as u32;
        vertices.push(Vertex {
            position: [0.0, -half_height, 0.0],
            normal: [0.0, -1.0, 0.0],
        });

        for i in 0..=segments {
            let theta = 2.0 * PI * i as f32 / segments as f32;
            let x = theta.cos();
            let z = theta.sin();
            vertices.push(Vertex {
                position: [x * radius, -half_height, z * radius],
                normal: [0.0, -1.0, 0.0],
            });
        }

        for i in 0..segments {
            indices.push(base_center_idx);
            indices.push(base_center_idx + 1 + i + 1);
            indices.push(base_center_idx + 1 + i);
        }

        let top_center_idx = vertices.len() as u32;
        vertices.push(Vertex {
            position: [0.0, half_height, 0.0],
            normal: [0.0, 1.0, 0.0],
        });

        for i in 0..=segments {
            let theta = 2.0 * PI * i as f32 / segments as f32;
            let x = theta.cos();
            let z = theta.sin();
            vertices.push(Vertex {
                position: [x * radius, half_height, z * radius],
                normal: [0.0, 1.0, 0.0],
            });
        }

        for i in 0..segments {
            indices.push(top_center_idx);
            indices.push(top_center_idx + 1 + i);
            indices.push(top_center_idx + 1 + i + 1);
        }

        Self::from_data(device, &vertices, &indices)
    }

    pub fn create_bone_transform(start: Vec3, end: Vec3) -> glam::Mat4 {
        let direction = end - start;
        let length = direction.length();

        if length < 0.0001 {
            return glam::Mat4::from_translation(start);
        }

        let up = direction.normalize();
        let right = if up.y.abs() < 0.999 {
            Vec3::Y.cross(up).normalize()
        } else {
            Vec3::X.cross(up).normalize()
        };
        let forward = up.cross(right);

        let center = (start + end) / 2.0;

        glam::Mat4::from_cols(
            right.extend(0.0),
            (up * length).extend(0.0),
            forward.extend(0.0),
            center.extend(1.0),
        )
    }

    pub fn wireframe_sphere(device: &wgpu::Device, radius: f32, segments: u32, rings: u32) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for ring in 0..=rings {
            let phi = PI * ring as f32 / rings as f32;
            let r = radius * phi.sin();
            let y = radius * phi.cos();

            let base_idx = vertices.len() as u32;
            for seg in 0..=segments {
                let theta = 2.0 * PI * seg as f32 / segments as f32;
                vertices.push(Vertex {
                    position: [r * theta.cos(), y, r * theta.sin()],
                    normal: [0.0, 1.0, 0.0],
                });

                if seg < segments {
                    indices.push(base_idx + seg);
                    indices.push(base_idx + seg + 1);
                }
            }
        }

        for seg in 0..segments {
            let theta = 2.0 * PI * seg as f32 / segments as f32;
            let base_idx = vertices.len() as u32;

            for ring in 0..=rings {
                let phi = PI * ring as f32 / rings as f32;
                let r = radius * phi.sin();
                let y = radius * phi.cos();
                vertices.push(Vertex {
                    position: [r * theta.cos(), y, r * theta.sin()],
                    normal: [0.0, 1.0, 0.0],
                });

                if ring < rings {
                    indices.push(base_idx + ring);
                    indices.push(base_idx + ring + 1);
                }
            }
        }

        Self::from_line_data(device, &vertices, &indices)
    }

    pub fn wireframe_box(device: &wgpu::Device, half_extents: Vec3) -> Self {
        let hx = half_extents.x;
        let hy = half_extents.y;
        let hz = half_extents.z;

        let vertices = vec![
            Vertex {
                position: [-hx, -hy, -hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [hx, -hy, -hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [hx, hy, -hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [-hx, hy, -hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [-hx, -hy, hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [hx, -hy, hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [hx, hy, hz],
                normal: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [-hx, hy, hz],
                normal: [0.0, 0.0, 0.0],
            },
        ];

        let indices = vec![
            0, 1, 1, 2, 2, 3, 3, 0, 
            4, 5, 5, 6, 6, 7, 7, 4, 
            0, 4, 1, 5, 2, 6, 3, 7,
        ];

        Self::from_line_data(device, &vertices, &indices)
    }

    pub fn from_line_data(device: &wgpu::Device, vertices: &[Vertex], indices: &[u32]) -> Self {
        use wgpu::util::DeviceExt;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Wireframe Vertex Buffer"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Wireframe Index Buffer"),
            contents: bytemuck::cast_slice(indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Self {
            vertex_buffer,
            index_buffer,
            index_count: indices.len() as u32,
        }
    }
}
