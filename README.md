# Pointcloud Editor
Pointcloud editor is a performance-oriented visualizer and editor of large pointcloud (`pcd`) files.

## Functionalities
Viz, rotate, crop.

## Commands
Pointcloud editor handles camera movements in orbit mode, with commands mapped similarly to `rviz`.
- **LMB** - *Orbit*: To orbit around the target, hold the left mouse button and move the mouse.
- **MMB** - *Movement*: To move the target, hold the middle mouse button and move the mouse.
- **RMB** - *Zoom*: To zoom closer to the target, hold the right mouse button and move the mouse.

## Build
```bash
cargo build -r
```

## Install
```bash
cargo install -r
```

## Running
With cargo:
```bash
cargo run -r <path_to_pointcloud.pcd>
```

With the executable:
```bash
./target/release/pointcloud_editor <path_to_pointcloud.pcd>
```

## Roadmap
Future features wishlist
- [ ] Display
  - [ ] Point color alternatives (intensity, axis color, etc...)
- [ ] Post-processing
  - [ ] Voxel filtering
  - [ ] Farthest point sampling
  - [ ] Isolated points removal
  - [ ] Sharpening
- [ ] Segmentation
  - [ ] Possibly color points based on segmentation results
- [ ] PNG export post-processing
  - [ ] Denoising


## Authors
* **Federico Gavioli** - [fgavioli](https://github.com/fgavioli) - _(Maintainer)_

