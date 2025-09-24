use half::f16;
use pcd_rs::{DynReader, DynRecord};
use std::{env, process::exit, sync::Arc};
use threecrate_io::RobustPcdReader;
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
    x: Vec<f16>,
    y: Vec<f16>,
    z: Vec<f16>,
    intensity: Vec<u8>,
    min_coord: glam::Vec3,
    max_coord: glam::Vec3,
    min_intensity: u8,
    max_intensity: u8,
}

fn parse_pcd(
    mut pcd_data: (threecrate_io::PcdHeader, Vec<threecrate_io::pcd::PcdPoint>),
) -> PointCloud {
    let has_intensity = pcd_data
        .0
        .fields
        .iter()
        .any(|f| f.name.to_lowercase() == "intensity");

    // Pre-allocate vectors for Structure of Arrays
    let num_points = pcd_data.1.len();
    let mut x_coords = Vec::with_capacity(num_points);
    let mut y_coords = Vec::with_capacity(num_points);
    let mut z_coords = Vec::with_capacity(num_points);
    let mut intensities = Vec::with_capacity(num_points);

    // Temporary vectors to calculate bounds
    let mut x_f32 = Vec::with_capacity(num_points);
    let mut y_f32 = Vec::with_capacity(num_points);
    let mut z_f32 = Vec::with_capacity(num_points);
    let mut intensity_f32 = Vec::with_capacity(num_points);

    // Parse and collect data, consuming the original data
    for point in pcd_data.1.drain(..) {
        let x = match &point.get("x").unwrap()[0] {
            threecrate_io::PcdValue::F32(val) => *val,
            _ => panic!("Expected Float32 for x coordinate"),
        };
        let y = match &point.get("y").unwrap()[0] {
            threecrate_io::PcdValue::F32(val) => *val,
            _ => panic!("Expected Float32 for y coordinate"),
        };
        let z = match &point.get("z").unwrap()[0] {
            threecrate_io::PcdValue::F32(val) => *val,
            _ => panic!("Expected Float32 for z coordinate"),
        };
        let intensity = if has_intensity {
            match &point.get("intensity").unwrap()[0] {
                threecrate_io::PcdValue::F32(val) => *val,
                _ => panic!("Expected Float32 for intensity"),
            }
        } else {
            0.0
        };

        x_f32.push(x);
        y_f32.push(y);
        z_f32.push(z);
        intensity_f32.push(intensity);
    }

    // Calculate bounds from f32 data
    let min_x = x_f32.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let max_x = x_f32.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
    let min_y = y_f32.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let max_y = y_f32.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
    let min_z = z_f32.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let max_z = z_f32.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));

    let min_intensity_f32 = intensity_f32.iter().fold(f32::INFINITY, |a, &b| a.min(b));
    let max_intensity_f32 = intensity_f32
        .iter()
        .fold(f32::NEG_INFINITY, |a, &b| a.max(b));

    // Normalize intensity to 0-255 range and convert to appropriate types
    let intensity_range = max_intensity_f32 - min_intensity_f32;
    let intensity_scale = if intensity_range > 0.0 {
        255.0 / intensity_range
    } else {
        0.0
    };

    for i in 0..num_points {
        x_coords.push(f16::from_f32(x_f32[i]));
        y_coords.push(f16::from_f32(y_f32[i]));
        z_coords.push(f16::from_f32(z_f32[i]));

        // Normalize and convert intensity to u8
        let normalized_intensity = if intensity_range > 0.0 {
            ((intensity_f32[i] - min_intensity_f32) * intensity_scale).clamp(0.0, 255.0) as u8
        } else {
            0u8
        };
        intensities.push(normalized_intensity);
    }

    PointCloud {
        x: x_coords,
        y: y_coords,
        z: z_coords,
        intensity: intensities,
        min_coord: glam::Vec3::new(min_x, min_y, min_z),
        max_coord: glam::Vec3::new(max_x, max_y, max_z),
        min_intensity: 0u8,   // Always 0 after normalization
        max_intensity: 255u8, // Always 255 after normalization
    }
}

fn load_pcd(filename: &str) -> Option<PointCloud> {
    println!("Loading {}\n", filename);
    let start = std::time::Instant::now();
    let res = RobustPcdReader::read_pcd_file(&filename);
    let load_time = start.elapsed().as_millis();
    if res.is_err() {
        println!("Failed to load file {}: {}", filename, res.err().unwrap());
        exit(-1);
    }

    let pcd_data = res.unwrap();
    println!(
        "[Read done, {} ms : {} points]",
        load_time,
        pcd_data.1.len()
    );
    print!("Available dimensions: ");
    for field in pcd_data.0.fields.iter() {
        print!("{} ", field.name);
    }
    println!();
    println!();

    let pointcloud = parse_pcd(pcd_data);

    println!(
        "PointCloud summary:\n- Points: {}\n- X: min {} max {}\n- Y: min {} max {}\n- Z: min {} max {}\n- Intensity: min {} max {}",
        pointcloud.x.len(),
        pointcloud.min_coord.x,
        pointcloud.max_coord.x,
        pointcloud.min_coord.y,
        pointcloud.max_coord.y,
        pointcloud.min_coord.z,
        pointcloud.max_coord.z,
        pointcloud.min_intensity,
        pointcloud.max_intensity
    );
    Some(pointcloud)
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
        min_coord: glam::Vec3::ZERO,
        max_coord: glam::Vec3::ZERO,
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
                f16::from_f32(values[0])
            } else {
                eprintln!("Expected F32 for x field. Found different type.");
                continue;
            });
        pointcloud
            .y
            .push(if let pcd_rs::Field::F32(ref values) = field_y {
                f16::from_f32(values[0])
            } else {
                eprintln!("Expected F32 for y field. Found different type.");
                continue;
            });
        pointcloud
            .z
            .push(if let pcd_rs::Field::F32(ref values) = field_z {
                f16::from_f32(values[0])
            } else {
                eprintln!("Expected F32 for z field. Found different type.");
                continue;
            });
        if has_intensity {
            pointcloud
                .intensity
                .push(if let pcd_rs::Field::F32(ref values) = field_intensity {
                    // Normalize intensity to 0-255 range
                    let norm_intensity = (values[0].clamp(0.0, 1.0) * 255.0) as u8;
                    norm_intensity
                } else {
                    0u8
                });
        } else {
            pointcloud.intensity.push(0u8);
        }
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

    // if pointcloud.is_none() {
    //     println!("Failed to load point cloud");
    //     return;
    // }
    // let pointcloud = pointcloud.unwrap();

    // println!("Starting renderer...");
    // let event_loop = EventLoop::new().unwrap();
    // let mut app = App::new(pointcloud);
    // event_loop.run_app(&mut app).unwrap();
}
