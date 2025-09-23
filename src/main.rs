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

struct Point {
    x: f32,
    y: f32,
    z: f32,
    i: f32,
}

struct PointCloud {
    points: Vec<Point>,
    min_coord: glam::Vec3,
    max_coord: glam::Vec3,
    min_intensity: f32,
    max_intensity: f32,
}

fn parse_pcd(
    pcd_data: (threecrate_io::PcdHeader, Vec<threecrate_io::pcd::PcdPoint>),
) -> PointCloud {
    // Move point cloud to our custom pointcloud struct
    let mut parsed_points = Vec::new();
    let has_intensity = pcd_data
        .0
        .fields
        .iter()
        .any(|f| f.name.to_lowercase() == "intensity");
    pcd_data.1.iter().for_each(|point| {
        parsed_points.push(Point {
            x: match &point.get("x").unwrap()[0] {
                threecrate_io::PcdValue::F32(val) => *val,
                _ => panic!("Expected Float32 for x coordinate"),
            },
            y: match &point.get("y").unwrap()[0] {
                threecrate_io::PcdValue::F32(val) => *val,
                _ => panic!("Expected Float32 for y coordinate"),
            },
            z: match &point.get("z").unwrap()[0] {
                threecrate_io::PcdValue::F32(val) => *val,
                _ => panic!("Expected Float32 for z coordinate"),
            },
            i: if has_intensity {
                match &point.get("intensity").unwrap()[0] {
                    threecrate_io::PcdValue::F32(val) => *val,
                    _ => panic!("Expected Float32 for intensity"),
                }
            } else {
                0.0
            },
        });
    });
    PointCloud {
        min_coord: glam::Vec3::new(
            parsed_points
                .iter()
                .map(|p| p.x)
                .fold(f32::INFINITY, f32::min),
            parsed_points
                .iter()
                .map(|p| p.y)
                .fold(f32::INFINITY, f32::min),
            parsed_points
                .iter()
                .map(|p| p.z)
                .fold(f32::INFINITY, f32::min),
        ),
        max_coord: glam::Vec3::new(
            parsed_points
                .iter()
                .map(|p| p.x)
                .fold(f32::NEG_INFINITY, f32::max),
            parsed_points
                .iter()
                .map(|p| p.y)
                .fold(f32::NEG_INFINITY, f32::max),
            parsed_points
                .iter()
                .map(|p| p.z)
                .fold(f32::NEG_INFINITY, f32::max),
        ),
        min_intensity: parsed_points
            .iter()
            .map(|p| p.i)
            .fold(f32::INFINITY, f32::min),
        max_intensity: parsed_points
            .iter()
            .map(|p| p.i)
            .fold(f32::NEG_INFINITY, f32::max),
        points: parsed_points,
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
        pointcloud.points.len(),
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

    let pointcloud = load_pcd(&filename);
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
