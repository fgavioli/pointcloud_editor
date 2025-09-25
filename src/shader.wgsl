// Vertex shader

struct CameraUniform {
    view_proj: mat4x4<f32>,
}

struct CropUniform {
    min_bounds: vec3<f32>,
    max_bounds: vec3<f32>,
}

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(0) @binding(1)
var<uniform> crop_bounds: CropUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) world_position: vec3<f32>,
}

@vertex
fn vs_main(
    model: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = model.color;
    out.world_position = model.position;
    out.clip_position = camera.view_proj * vec4<f32>(model.position, 1.0);
    return out;
}

// Fragment shader

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Check if the point is inside the crop bounds
    let inside_crop = all(in.world_position >= crop_bounds.min_bounds) &&
                     all(in.world_position <= crop_bounds.max_bounds);

    // Set alpha based on whether point is inside crop bounds
    let alpha = select(0.05, 1.0, inside_crop);

    // Set color: original color if inside crop bounds, grey if outside
    let grey_color = vec3<f32>(0.1, 0.1, 0.1);
    let final_color = select(grey_color, in.color, inside_crop);

    return vec4<f32>(final_color, alpha);
}
