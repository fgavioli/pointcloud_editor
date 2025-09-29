mod camera;
mod camera_controller;
mod gui;
mod pointcloud;
mod renderer;
mod app;

use std::env;
use pointcloud::{load_pcd_streaming, PointCloud};
use winit::event_loop::EventLoop;

use crate::app::App;

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
        println!("Usage: pointcloud_editor [path_to_pcd_file]");
        return;
    }

    let pointcloud = load_pcd_streaming(&filename, true);
    if pointcloud.is_none() {
        println!("Failed to load point cloud");
        return;
    }
    let pointcloud = pointcloud.unwrap();
    let event_loop = EventLoop::new().unwrap();
    let mut app = App::new(pointcloud);
    event_loop.run_app(&mut app).unwrap();
}