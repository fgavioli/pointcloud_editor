/// Point cloud loading, processing and utilities

use glam::{Mat3, Quat, Vec3};
use pcd_rs::DynReader;
use std::io::Write;

// Fit a plane to the input cloud via PCA
pub fn planefit_pca(points: &[Vec3]) -> Quat {
    if points.len() < 3 {
        return Quat::IDENTITY;
    }
    let centroid = points.iter().copied().reduce(|a, b| a + b).unwrap() / (points.len() as f32);
    let mut cov = Mat3::ZERO;
    for &p in points {
        let r = p - centroid;
        cov += Mat3::from_cols(r * r.x, r * r.y, r * r.z);
    }
    cov /= points.len() as f32;

    let epsilon = 1e-6;
    let inv_cov = cov + Mat3::from_diagonal(Vec3::splat(epsilon));
    let inv_cov = inv_cov.inverse();

    let mut normal = Vec3::new(1.0, 0.0, 0.0);
    for _ in 0..20 {
        normal = (inv_cov * normal).normalize();
    }

    // flip plane if normal points down
    if normal.z < 0.0 {
        normal = -normal;
    }

    let target = Vec3::Z;
    let rot = if normal.abs_diff_eq(target, 1e-6) {
        if normal.dot(target) > 0.0 {
            Quat::IDENTITY
        } else {
            Quat::from_axis_angle(Vec3::X, std::f32::consts::PI)
        }
    } else {
        let axis = normal.cross(target).normalize();
        let angle = normal.dot(target).clamp(-1.0, 1.0).acos();
        Quat::from_axis_angle(axis, angle)
    };

    rot
}

pub fn align_to_ground(
    pointcloud: &PointCloud,
    ground_point: glam::Vec3,
    radius: glam::Vec3,
) -> PointCloud {
    let planefit_points: Vec<glam::Vec3> = pointcloud
        .points
        .iter()
        .filter(|&point| {
            let dist = *point - ground_point;
            dist.x.abs() <= radius.x && dist.y.abs() <= radius.y && dist.z.abs() <= radius.z
        })
        .copied()
        .collect();

    let rotation = planefit_pca(&planefit_points);

    let aligned_points: Vec<glam::Vec3> = pointcloud
        .points
        .iter()
        .map(|&original_point| rotation * original_point)
        .collect();

    let min_coord = aligned_points
        .iter()
        .fold(glam::Vec3::splat(f32::INFINITY), |acc, &point| {
            acc.min(point)
        });
    let max_coord = aligned_points
        .iter()
        .fold(glam::Vec3::splat(f32::NEG_INFINITY), |acc, &point| {
            acc.max(point)
        });

    let size = max_coord - min_coord;

    PointCloud {
        points: aligned_points,
        intensity: pointcloud.intensity.clone(),
        min_coord,
        max_coord,
        size,
        min_intensity: pointcloud.min_intensity,
        max_intensity: pointcloud.max_intensity,
    }
}


/// Find the closest point in the point cloud to a ray
pub fn find_closest_point_to_ray(
    pointcloud: &PointCloud,
    ray_origin: glam::Vec3,
    ray_direction: glam::Vec3,
    max_distance: f32,
) -> Option<glam::Vec3> {
    let mut closest_point = None;
    let mut min_distance = f32::MAX;

    for &point in &pointcloud.points {
        // Calculate distance from point to ray using cross product
        let to_point = point - ray_origin;
        let projection_length = to_point.dot(ray_direction);

        // Only consider points in front of the camera
        if projection_length > 0.0 {
            let projection = ray_origin + ray_direction * projection_length;
            let distance_to_ray = point.distance(projection);

            // Check if this point is closer to the ray and within max distance
            if distance_to_ray < min_distance && distance_to_ray < max_distance {
                min_distance = distance_to_ray;
                closest_point = Some(point);
            }
        }
    }

    closest_point
}


#[derive(Clone)]
pub struct PointCloud {
    pub points: Vec<glam::Vec3>,
    pub intensity: Vec<u8>,
    pub min_coord: glam::Vec3,
    pub max_coord: glam::Vec3,
    pub size: glam::Vec3, // 3D size (extent) of the point cloud
    pub min_intensity: u8,
    pub max_intensity: u8,
}

pub fn load_pcd_streaming(filename: &str) -> Option<PointCloud> {
    let start = std::time::Instant::now();
    let reader = match DynReader::open(filename).ok() {
        Some(r) => r,
        None => {
            println!("{} not found.", filename);
            return None;
        }
    };

    // Check if we can get the total number of points from metadata
    let total_points = reader.meta().width * reader.meta().height;
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
        points: Vec::new(),
        intensity: Vec::new(),
        min_coord: glam::Vec3::splat(f32::INFINITY),
        max_coord: glam::Vec3::splat(f32::NEG_INFINITY),
        size: glam::Vec3::ZERO,
        min_intensity: 0u8,
        max_intensity: 255u8,
    };

    // Progress tracking variables
    let mut points_loaded = 0usize;
    let update_interval = (total_points as usize) / 100;
    let mut last_progress_update = std::time::Instant::now();

    println!("Opened {}.", filename);
    println!(
        "Available fields: {:?}",
        schema.iter().map(|f| f.name.clone()).collect::<Vec<_>>()
    );

    // Print initial progress bar
    print!("Loading: [");
    for _ in 0..25 {
        print!(" ");
    }
    print!("] 0%\r");
    std::io::stdout().flush().unwrap();

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

        let x_val = if let pcd_rs::Field::F32(ref values) = field_x {
            values[0]
        } else {
            eprintln!("Expected F32 for x field. Found different type.");
            continue;
        };
        let y_val = if let pcd_rs::Field::F32(ref values) = field_y {
            values[0]
        } else {
            eprintln!("Expected F32 for y field. Found different type.");
            continue;
        };
        let z_val = if let pcd_rs::Field::F32(ref values) = field_z {
            values[0]
        } else {
            eprintln!("Expected F32 for z field. Found different type.");
            continue;
        };
        pointcloud.points.push(glam::Vec3::new(x_val, y_val, z_val));

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

        // Update progress tracking
        points_loaded += 1;

        // Update progress bar periodically
        if points_loaded % update_interval == 0 || last_progress_update.elapsed().as_millis() > 100
        {
            let progress_percent = ((points_loaded as f64 / total_points as f64) * 100.0) as usize;
            let progress_percent = progress_percent.min(100);
            let filled_bars = (progress_percent * 25) / 100;

            print!("Loading : [");
            for i in 0..25 {
                if i < filled_bars {
                    print!("=");
                } else if i == filled_bars {
                    print!(">");
                } else {
                    print!(" ");
                }
            }
            print!("] {}% ({} points)\r", progress_percent, points_loaded);
            std::io::stdout().flush().unwrap();
            last_progress_update = std::time::Instant::now();
        }
    }

    // Clear the progress line
    print!("\r");
    for _ in 0..80 {
        print!(" ");
    }
    print!("\r");

    // Compute bounding box
    pointcloud.min_coord = pointcloud
        .points
        .iter()
        .fold(glam::Vec3::splat(f32::INFINITY), |min, p| min.min(*p));
    pointcloud.max_coord = pointcloud
        .points
        .iter()
        .fold(glam::Vec3::splat(f32::NEG_INFINITY), |max, p| max.max(*p));

    // Calculate 3D size (extent) of the point cloud
    pointcloud.size = pointcloud.max_coord - pointcloud.min_coord;

    // rescale intensity to 0-255
    if has_intensity {
        let (min_intensity, max_intensity) = pointcloud
            .intensity
            .iter()
            .fold((u8::MAX, u8::MIN), |(min, max), &val| {
                (min.min(val), max.max(val))
            });
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
        pointcloud.points.len()
    );

    Some(pointcloud)
}

pub fn export_pcd(filename: &str, pointcloud: &PointCloud) -> Option<()> {
    // /// Export point cloud to PCD file with pcd-rs
    // let mut file = std::fs::File::create(filename).ok()?;
    // let header = pcd_rs::Header {
    //     version: Some("0.7".to_string()),
    //     fields: vec![
    //         pcd_rs::FieldDef {name:"x".to_string(),datatype:pcd_rs::Datatype::F32,count:1, kind: todo!() },
    //         pcd_rs::FieldDef { name: "y".to_string(), datatype: pcd_rs::Datatype::F32, count: 1 },
    //         pcd_rs::FieldDef { name: "z".to_string(), datatype: pcd_rs::Datatype::F32, count: 1 },
    //         pcd_rs::FieldDef { name: "intensity".to_string(), datatype: pcd_rs::Datatype::F32, count: 1 },
    //     ],
    // };
    // pcd_rs::write_pcd(&mut file, &header, pointcloud).ok()?;
    Some(())
}