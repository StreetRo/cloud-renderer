#version 330

// Uniform variables are constant throughout the entire shader
// execution. They are also read-only to enable parallelization.
uniform vec3 u_cam_pos;
uniform mat4 u_inv_projectionview;

in vec4 in_position;

out vec4 v_position;
out vec3 v_origin;
out vec3 v_raydir;

void main() {
  gl_Position = vec4( in_position.xy, 0, 1 );
  v_raydir = normalize( (u_inv_projectionview * vec4( in_position.xy, 1, 1 )).xyz );
  v_origin = (vec4( u_cam_pos.xyz,  1 )).xyz;
}
