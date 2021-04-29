#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;

uniform mat3 u_c2w;

uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform sampler2D u_density_tex_1;

uniform vec3 u_bbox_min;
uniform vec3 u_bbox_max;

uniform vec3 u_rt_screen_norm; // direction camera is pointing

uniform float u_cloud_scale;
uniform vec3 u_cloud_offset;
uniform float u_density_thresh;
uniform float u_density_mult;
uniform int u_density_samples;

in vec4 v_position;
in vec4 v_normal;
in vec4 xy_pos;

out vec4 out_color;

vec2 rayBoxDst(vec3 boundsMin, vec3 boundsMax, vec3 rayOrigin, vec3 invRaydir);


float sampleDensity( vec3 pos ) {
    vec3 xyz = pos * u_cloud_scale * 0.001 + u_cloud_offset * 0.01;
    float d = texture( u_density_tex_1, vec2(xyz) ).x;

    return max( 0, d - u_density_thresh * 0.1 ) * u_density_mult;
}

void main() {
    // vec3 ray_orig = v_position.xyz;
    vec3 ray_orig = u_cam_pos;
    // vec3 ray_dir = normalize( u_rt_screen_norm );
    // vec3 ray_dir = vec3( -1, -1, 0 );
    // vec3 ray_dir = vec3( 0, 0, -1 );
    // vec3 ray_dir = vec3( -1, 1, 1 );
    // vec3 ray_dir = u_cam_pos + vec3( -1, 1, 0.01 );
    // vec3 ray_dir = normalize( u_cam_pos - vec3( v_position ) );
    vec3 ray_dir = normalize( v_position.xyz - u_cam_pos  );
    // vec3 ray_dir = normalize( v_position.xyz - u_cam_pos );

    // float val = distance( v_position.xyz, u_cam_pos );
    // out_color = vec4( val, 0, 0, 0.5 );

    // out_color = vec4( val, val, val, 1 );
    // out_color = vec4( u_rt_screen_norm, 0.5 );

    vec2 distances = rayBoxDst( u_bbox_min, u_bbox_max, ray_orig, ray_dir );
    float d_to_box = distances.x;
    float d_in_box = distances.y;

    if ( d_in_box > 0 ) {
        //float d = 0.f;
        //float d_travd = 0.f;
        //int n = 0;
        //float step_size = d_in_box / u_density_samples;
        //while ( d_travd < d_in_box ) {
//
        //    vec3 v = u_cam_pos + ( d_travd + d_to_box ) * ray_dir;
//
        //    d += sampleDensity( v ) * step_size;
//
        //    d_travd += step_size;
        //    n += 1;
        //}
//
        //d = exp( -( 1 - d ) );
        //vec4 t = texture( u_density_tex, vec2(v_position.x, v_position.y)  );
        out_color = vec4(1, 1, 1, 1);
        //out_color = vec4( d, d, d, 1 );
        // out_color = u_color;
    } else {
        // out_color = vec4( u_cam_pos, 1 ) - v_position; out_color.a = 1;
        
        // out_color = vec4( v_position.xyz, 0.5 );
        vec4 t = texture( u_density_tex_1, xy_pos.xy );
        //vec4 t = texture( u_density_tex, v_position.xy );
        //out_color = vec4(t.r, t.g, t.b, t.a);
        out_color = vec4(1.0, 1.0, 1.0, t.r);
        //vec4 t = texture( u_density_tex, vec3(v_position.x, v_position.y, 0.0)  );
        //out_color = vec4( 0.7, .98, .96, 0.5 );
    }
}



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

