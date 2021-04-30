#version 330

// Uniform variables are constant throughout the entire shader
// execution. They are also read-only to enable parallelization.
uniform vec3 u_cam_pos;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_inv_view;
uniform mat4 u_projection;
uniform mat4 u_inv_projection;
uniform mat4 u_inv_viewprojection;
uniform mat4 u_inv_projectionview;
uniform mat4 u_view_projection;

uniform float u_near;
uniform float u_far;

// In a vertex shader, the "in" variables are read-only per-vertex 
// properties. An example of this was shown in the rasterizer project, 
// where each vertex had an associated "color" or "uv" value which we 
// would later interpolate using barycentric coordinates.
in vec4 in_position;

// In a vertex shader, the "out" variables are per-vertex properties
// that are read/write. These properties allow us to communicate
// information from the vertex shader to the fragment shader.
// That is, in the linked fragment shader, these values become the 
// "in" variables.
out vec4 v_position;
out vec3 v_origin;
out vec3 v_raydir;
out vec3 v_ray;
out mat4 v_inv_viewprojection;

// Every shader features a "main" function.
// This is typically where we write to the "out" variables that the
// fragment shaders get to use. It is also where "gl_Position" is set,
// which is the final screen-space location of this vertex which the
// GPU's triangle rasterizer takes in.
void main() {
  gl_Position = vec4( in_position.xy, 0, 1 );
  // v_position = in_position;
  // v_origin = (u_inv_viewprojection * vec4(in_position.xy, -1.0, 1.0) * u_near).xyz;
  // v_raydir = normalize( (u_inv_viewprojection * vec4(in_position.xy * (u_far - u_near), u_far + u_near, u_far - u_near)).xyz );
  // v_inv_viewprojection = u_inv_viewprojection;
  // v_position = u_model * in_position;
  // v_raydir = normalize( vec3( u_inv_viewprojection * in_position ) );
  // v_ray = (u_inv_viewprojection * (vec4(in_position.xy, 1.0, 1.0) * u_far - vec4(in_position.xy, -1.0, 1.0) * u_near)).xyz;

  // v1 = u_view_projection * vec4( in_position.xy,  )



  // v_position = u_view_projection * u_model * in_position;
  // v_position = in_position;
  // v_raydir = (u_inv_viewprojection * in_position).xyz;
  v_raydir = (u_inv_viewprojection * vec4( u_cam_pos, 1 )).xyz;
  v_origin = (u_inv_view * vec4( in_position.xy, 2, 1 )).xyz;



  // gl_Position = vec4( in_position.xy, 0, 1 );

  // v_normal = normalize(u_model * in_normal);
  // v_uv = in_uv;
  // v_tangent = normalize(u_model * in_tangent);
  
  // The final screen-space location of this vertex which the
  // GPU's triangle rasterizer takes in.
  // gl_Position = u_projection * u_model * in_position;
  // gl_Position = u_view_projection * u_model * in_position;
}
