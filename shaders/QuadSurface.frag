#version 330

uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform sampler3D u_density_tex;
uniform sampler2D u_noise;

uniform vec3 u_bbox_min;
uniform vec3 u_bbox_max;

uniform float u_cloud_scale;
uniform vec3 u_cloud_offset;
uniform float u_density_thresh;
uniform float u_density_mult;
uniform int u_density_samples;

in vec4 v_position;
in vec3 v_origin;
in vec3 v_raydir;

out vec4 out_color;

vec2 rayBoxDst(vec3 boundsMin, vec3 boundsMax, vec3 rayOrigin, vec3 invRaydir);

float scale( float v, float lo, float ho, float ln, float hn ) {
    return ln + ( (v - lo) * (hn - ln) ) / (ho - lo);
}

float sampleDensity( vec4 pos ) {
    // vec3 xyz = pos.xyz;
    // vec3 xyz = pos.xyz + u_cloud_offset * 0.1;
    vec3 xyz = pos.xyz * u_cloud_scale * 0.1 + u_cloud_offset * 0.1;
    vec4 tex = texture( u_noise, ( xyz.xy + 0.5 ) );

    // Generate higher frequency worley using same texture
    float freq2 = texture( u_noise, 4 * ( xyz.xy + 0.5 ) ).g;
    float freq3 = texture( u_noise, 16 * ( xyz.xy + 0.5 ) ).g;

    // Fractional Brownion Motion to decay the higher freq noise
    float shape_noise = tex.g * (0.625) + freq2 * (0.125) + freq3 * (0.0625);
    shape_noise = shape_noise - 1;
    shape_noise = scale( tex.r, shape_noise, 1, 0, 1 );

    // return shape_noise;
    // return freq3;
    // return 1 - d;
    return max( 0, shape_noise - u_density_thresh * 0.1 ) * u_density_mult;
}

vec3 intersect( vec3 boxmin, vec3 boxmax, vec3 o, vec3 d ) {
  float tx1 = ( boxmin.x - o.x ) / d.x;
  float tx2 = ( boxmax.x - o.x ) / d.x;

  // y-axis
  float ty1 = ( boxmin.y - o.y ) / d.y;
  float ty2 = ( boxmax.y - o.y ) / d.y;

  // z-axis
  float tz1 = ( boxmin.z - o.z ) / d.z;
  float tz2 = ( boxmax.z - o.z ) / d.z;

  float txmin = min( tx1, tx2 );
  float txmax = max( tx1, tx2 );
  float tymin = min( ty1, ty2 );
  float tymax = max( ty1, ty2 );
  float tzmin = min( tz1, tz2 );
  float tzmax = max( tz1, tz2 );

  float tmin = max( max( txmin, tymin ), tzmin );
  float tmax = min( min( txmax, tymax), tzmax );

  if ( tmin > tmax || tmax < 0 ) {
    return vec3(-1, -1, -1); }

  // FIXME: Setting these breaks importance sampling of dae/sky/CBbunny.dae
  // t0 = tmin;
  // t1 = tmax;

  return vec3( tmin, tmax, 1 );
}

void main() {
    vec3 o = v_origin;
    vec3 d = v_raydir;

    vec3 dists = intersect( u_bbox_min, u_bbox_max, o, d );
    if ( dists.z == 1 ) {
      float d_to_box = dists.x;
      float d_in_box = dists.y - dists.x;

      float val = 0.f;
      float d_travd = 0.f;
      int n = 0;
      float step_size = d_in_box / u_density_samples;
      while ( d_travd < d_in_box ) {
          vec3 v = o + ( d_travd + d_to_box ) * d;

          val += sampleDensity( vec4( v, 1 ) ) * step_size;

          d_travd += step_size;
          n += 1;
      }

      val = exp( -( 1 - val ) );
      out_color = vec4( 1, 1, 1, val );
    }
}

