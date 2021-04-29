#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;

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
in vec4 v_normal;

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
    float shape_noise = tex.g * (2/3) + freq2 * (1/2) + freq3 * (1/4);
    shape_noise = shape_noise - 1;
    shape_noise = scale( tex.r, shape_noise, 1, 0, 1 );

    // return shape_noise;
    // return freq3;
    // return 1 - d;
    return max( 0, shape_noise - u_density_thresh * 0.1 ) * u_density_mult;
}

void main() {

    float d = sampleDensity( v_position );

    out_color = vec4( 1, 1, 1, d );

    // vec4 tex = texture( u_noise, ( v_position.xy + 0.5 ) );
    // out_color = vec4( tex.r, 0, 0.5 * tex.g, 1 );


    // out_color = vec4( tex.x, tex.y, 0, 1 );
    // out_color = vec4( v_position.x + 0.25, v_position.y + 0.25, tex.x, 1 );
    // out_color = vec4( v_position.x + 0.5, v_position.y + 0.5, 0, 1 );
}


// void main() {
    // vec3 wo = normalize( u_cam_pos - vec3( 2. * v_position ) );
    // vec2 distances = rayBoxDst( u_bbox_min, u_bbox_max, u_cam_pos, -wo );
    // float d_to_box = distances.x;
    // float d_in_box = distances.y;

    // float d = 0.f;
    // float d_travd = 0.f;
    // int n = 0;
    // float step_size = d_in_box / u_density_samples;
    // while ( d_travd < d_in_box ) {

        // vec3 v = u_cam_pos + ( d_travd + d_to_box ) * -wo;

        // d += sampleDensity( v ) * step_size;

        // d_travd += step_size;
        // n += 1;
    // }

    // d = exp( -( d ) );
    // out_color = vec4( d, d, d, d );
// }


/* reference: https://github.com/SebLague/Clouds/blob/master/Assets/Scripts/Clouds/Shaders/Clouds.shader */
vec2 rayBoxDst(vec3 boundsMin, vec3 boundsMax, vec3 rayOrigin, vec3 invRaydir) {
    // Adapted from: http://jcgt.org/published/0007/03/04/
    vec3 t0 = (boundsMin - rayOrigin) * invRaydir;
    vec3 t1 = (boundsMax - rayOrigin) * invRaydir;
    vec3 tmin = min(t0, t1);
    vec3 tmax = max(t0, t1);
    
    float dstA = max(max(tmin.x, tmin.y), tmin.z);
    float dstB = min(tmax.x, min(tmax.y, tmax.z));

    // CASE 1: ray intersects box from outside (0 <= dstA <= dstB)
    // dstA is dst to nearest intersection, dstB dst to far intersection

    // CASE 2: ray intersects box from inside (dstA < 0 < dstB)
    // dstA is the dst to intersection behind the ray, dstB is dst to forward intersection

    // CASE 3: ray misses box (dstA > dstB)

    float dstToBox = max(0, dstA);
    float dstInsideBox = max(0, dstB - dstToBox);
    return vec2(dstToBox, dstInsideBox);
}

