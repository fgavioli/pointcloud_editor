use pcd_rs::DynReader;
use std::{env, sync::Arc};
use winit::{
    application::ApplicationHandler,
    event::*,
    event_loop::{ActiveEventLoop, EventLoop},
    window::{Window, WindowId},
};
mod renderer;
use renderer::Renderer;

struct App {
    pointcloud: PointCloud,
    renderer: Option<Renderer>,
    window: Option<Arc<Window>>,
}

impl App {
    fn new(pointcloud: PointCloud) -> Self {
        Self {
            pointcloud,
            renderer: None,
            window: None,
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window_attributes = Window::default_attributes()
            .with_title("Point Cloud Editor")
            .with_inner_size(winit::dpi::LogicalSize::new(1024, 768));

        let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
        let renderer = pollster::block_on(Renderer::new(window.clone(), &self.pointcloud));

        self.window = Some(window);
        self.renderer = Some(renderer);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        if let Some(ref mut renderer) = self.renderer {
            if !renderer.input(&event) {
                match event {
                    WindowEvent::CloseRequested
                    | WindowEvent::KeyboardInput {
                        event:
                            KeyEvent {
                                logical_key:
                                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape),
                                state: ElementState::Pressed,
                                ..
                            },
                        ..
                    } => event_loop.exit(),
                    WindowEvent::Resized(physical_size) => {
                        renderer.resize(physical_size);
                    }
                    WindowEvent::RedrawRequested => {
                        renderer.update();
                        match renderer.render() {
                            Ok(_) => {}
                            Err(wgpu::SurfaceError::Lost) => renderer.resize(renderer.size),
                            Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                            Err(e) => eprintln!("{:?}", e),
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(ref window) = self.window {
            window.request_redraw();
        }
    }
}

struct PointCloud {
    // Structure of Arrays (SoA) for better memory efficiency
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    intensity: Vec<u8>,
    min_coord: [f32; 3],
    max_coord: [f32; 3],
    min_intensity: u8,
    max_intensity: u8,
}

fn load_pcd_streaming(filename: &str) -> Option<PointCloud> {
    println!("Loading {}\n", filename);
    let start = std::time::Instant::now();
    let reader = match DynReader::open(filename).ok() {
        Some(r) => r,
        None => {
            println!("{} not found.", filename);
            return None;
        }
    };

    // Discover schema at runtime
    let schema = reader.meta().field_defs.fields.clone();
    let x_index = match schema.iter().position(|f| f.name.to_lowercase() == "x") {
        Some(idx) => idx,
        None => {
            eprintln!("PCD file schema does not contain the x field.");
            return None;
        }
    };
    let y_index = match schema.iter().position(|f| f.name.to_lowercase() == "y") {
        Some(idx) => idx,
        None => {
            eprintln!("PCD file schema does not contain the y field.");
            return None;
        }
    };
    let z_index = match schema.iter().position(|f| f.name.to_lowercase() == "z") {
        Some(idx) => idx,
        None => {
            eprintln!("PCD file schema does not contain the z field.");
            return None;
        }
    };
    let mut has_intensity = true;
    let intensity_index = match schema
        .iter()
        .position(|f| f.name.to_lowercase() == "intensity")
    {
        Some(idx) => idx,
        None => {
            println!(
                "WARN: PCD file schema does not contain the intensity field. Defaulting to 0."
            );
            has_intensity = false;
            0
        }
    };

    let mut pointcloud = PointCloud {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        intensity: Vec::new(),
        min_coord: [0.0, 0.0, 0.0],
        max_coord: [0.0, 0.0, 0.0],
        min_intensity: 0u8,
        max_intensity: 255u8,
    };
    for record in reader {
        // Access by index~
        let point = match record {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Error reading record: {:?}", e);
                continue;
            }
        };
        let field_x = &point.0[x_index];
        let field_y = &point.0[y_index];
        let field_z = &point.0[z_index];
        let field_intensity = &point.0[intensity_index];

        // Extract specific types
        pointcloud
            .x
            .push(if let pcd_rs::Field::F32(ref values) = field_x {
                values[0]
            } else {
                eprintln!("Expected F32 for x field. Found different type.");
                continue;
            });
        pointcloud
            .y
            .push(if let pcd_rs::Field::F32(ref values) = field_y {
                values[0]
            } else {
                eprintln!("Expected F32 for y field. Found different type.");
                continue;
            });
        pointcloud
            .z
            .push(if let pcd_rs::Field::F32(ref values) = field_z {
                values[0]
            } else {
                eprintln!("Expected F32 for z field. Found different type.");
                continue;
            });
        if has_intensity {
            pointcloud
                .intensity
                .push(if let pcd_rs::Field::F32(ref values) = field_intensity {
                    // Normalize intensity to 0-255 range
                    values[0] as u8
                } else {
                    0u8
                });
        } else {
            pointcloud.intensity.push(0u8);
        }
    }

    // Compute bounding box
    pointcloud.min_coord = [
        pointcloud.x.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
        pointcloud.y.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
        pointcloud.z.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
    ];
    pointcloud.max_coord = [
        pointcloud.x.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b)),
        pointcloud.y.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b)),
        pointcloud.z.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b)),
    ];

    // rescale intensity to 0-255
    if has_intensity {
        let (min_intensity, max_intensity) = pointcloud
            .intensity
            .iter()
            .fold((u8::MAX, u8::MIN), |(min, max), &val| (min.min(val), max.max(val)));
        pointcloud.intensity = pointcloud
            .intensity
            .iter()
            .map(|&val| {
                ((val - min_intensity) as f32 / (max_intensity - min_intensity) as f32 * 255.0)
                    as u8
            })
            .collect();
    }

    let load_time = start.elapsed().as_millis();
    println!(
        "[Load done, {} ms : {} points]",
        load_time,
        pointcloud.x.len()
    );
    println!(
        "Available fields: {:?}",
        schema.iter().map(|f| f.name.clone()).collect::<Vec<_>>()
    );
    println!();
    println!();

    Some(pointcloud)
}

fn main() {
    env_logger::init();

    let mut filename = String::new();
    if env::args().len() > 2 {
        println!("Usage: pointcloud_editor [path_to_pcd_file]");
        return;
    }
    if env::args().len() > 1 {
        let arg_filename = env::args().nth(1).unwrap();
        if !arg_filename.is_empty() {
            filename = arg_filename;
        }
    } else {
        // Default to a sample file if no argument provided
        filename = "samples/kaist03_000020.pcd".to_string();
    }

    let pointcloud = load_pcd_streaming(&filename);
    if pointcloud.is_none() {
        println!("Failed to load point cloud");
        return;
    }
    let pointcloud = pointcloud.unwrap();

    println!("Starting renderer...");
    let event_loop = EventLoop::new().unwrap();
    let mut app = App::new(pointcloud);
    event_loop.run_app(&mut app).unwrap();
}