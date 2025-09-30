# Point Cloud Editor
![Point Cloud Editor](https://img.shields.io/badge/language-Rust-orange.svg)
![License](https://img.shields.io/badge/license-Apache2-blue.svg)
![Version](https://img.shields.io/badge/version-1.0.0-green.svg)

A performance-focused, Rust-based 3D point cloud visualizer and editor for processing large PCD files. Inspired by `pcl_viewer` and extended with point cloud editing tools.

**Visualizer:**
- Orbit-based 3D point cloud visualization.

**Point Cloud Processing:**
- PCA-based ground plane alignment
- 3D bounding box cropping

**Export:**
- **PCD export** - Save processed point clouds into `.pcd` files
- **PNG/YAML export** - Export flattened occupancy grid as ROS-compatible map files

## **Camera Controls**
- **Left Mouse Button (LMB)** - Orbit around target
- **Middle Mouse Button (MMB)** or **Shift + LMB** - Move target position
- **Right Mouse Button (RMB)** or **Mouse Wheel** - Zoom in/out

## Prerequisites
- Rust 1.70+
- WGSL-capable hardware

While ideally this program should be run on a discrete GPU, for smaller pointclouds a laptop integrated GPU is perfectly capable of running `pointcloud_editor` at 60+ FPS.

## Installation

### Release (Recommended)
Download the latest `.deb` package from [Releases](https://github.com/fgavioli/pointcloud_editor/releases):
- [Release 1.0](https://drive.google.com/file/d/1TdDt1wy7wOb5hEpOTlSOsO84CarHmyWa/view)

```bash
# On Ubuntu/Debian
sudo dpkg -i pointcloud_editor_1.0.0_amd64.deb
```

### From sources
```bash
# Clone the repository
git clone https://git.hipert.unimore.it/adx/utils/pointcloud_editor.git
cd pointcloud_editor

# Build and run in release mode
cargo run --release -- path/to/your/pointcloud.pcd
```

### Usage

```bash
# Run with a PCD file
pointcloud_editor samples/outdoor.pcd

# Or if built from source
cargo run --release -- samples/outdoor.pcd
```
## Roadmap

### **Display**
- [ ] Multiple color schemes (height-based, custom palettes)
- [ ] Point size adjustment
- [ ] Transparency controls
- [ ] Measurement tools (distance, area, volume)

### **Processing**
- [ ] Voxel grid downsampling
- [ ] Farthest point sampling
- [ ] Outlier removal algorithms
- [ ] Point cloud sharpening

### **Analysis**
- [ ] Color-coded segmentation display
- [ ] Surface reconstruction

### **Export**
- [ ] 3D format exports (PLY, OBJ)
- [ ] Occupancy grid post-processing

## Contributing
Contributions are welcome! Please feel free to submit pull requests or open issues for bug reports and feature requests.

## License

This project is licensed under the Apache2 License - see [LICENSE](LICENSE.md) for details.

## Authors
* **Federico Gavioli** - [fgavioli](https://github.com/fgavioli) - _(Maintainer)_
