# Pointcloud Editor
Pointcloud editor is a performance-oriented visualizer and editor of large pointcloud (`pcd`) files.

## Functionalities
Visualize, laign, crop and export large pointclouds.

## Commands
Pointcloud editor handles camera movements in orbit mode, with commands mapped similarly to `rviz`.
- **LMB** - *Orbit*: To orbit around the target, hold the left mouse button and move the mouse.
- **MMB** - *Movement*: To move the target, hold the middle mouse button and move the mouse.
- **RMB** - *Zoom*: To zoom closer to the target, hold the right mouse button and move the mouse.

## Install
Download the `dpkg` package of the latest [release](https://git.hipert.unimore.it/adx/utils/pointcloud_editor/releases).

Releases
- [Release 1.0](https://drive.google.com/file/d/1TdDt1wy7wOb5hEpOTlSOsO84CarHmyWa/view)

## Development
Clone the repo, `cd` into the directory and to build and run the project, run
```bash
cargo run -r <path_to_pointcloud_file.pcd>
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

