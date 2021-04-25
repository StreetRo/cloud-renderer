#include "clouds.h"

void Clouds::drawContents() {
  glEnable( GL_DEPTH_TEST );
  glEnable( GL_BLEND );
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  /*
   * Scene globals
   */
  Matrix4f model;
  model.setIdentity();

  Matrix4f view = getViewMatrix();
  Matrix4f projection = getProjectionMatrix();
  Matrix4f viewProjection = projection * view;

  Vector3D cam_pos = camera.position();

  /* Draw Bounding Box */
  {
    auto& user_shad = shader_map["WorleyPoints"];

    GLShader& shader = *(user_shad.nanogui_shader);
    shader.bind();

    // Prepare the camera projection matrix
    shader.setUniform("u_model", model);
    shader.setUniform("u_view_projection", viewProjection);

    shader.setUniform("u_cam_pos", Vector3f(cam_pos.x, cam_pos.y, cam_pos.z), false);

    if ( enableBoundingBoxDraw ){ drawBoundingBox( shader ); }
  }

  /* Draw Worley Points */
  {
    auto& user_shad = shader_map["WorleyPoints"];

    GLShader& shader = *(user_shad.nanogui_shader);
    shader.bind();

    // Prepare the camera projection matrix
    shader.setUniform("u_model", model);
    shader.setUniform("u_view_projection", viewProjection);

    shader.setUniform("u_cam_pos", Vector3f(cam_pos.x, cam_pos.y, cam_pos.z), false);

    if ( enableWorleyDraw ){ drawWorleyPoints( shader ); }
  }

  /* Draw Density Points */
  {
    auto& user_shad = shader_map["PointCloud"];

    GLShader& shader = *(user_shad.nanogui_shader);
    shader.bind();

    // Prepare the camera projection matrix
    shader.setUniform("u_model", model);
    shader.setUniform("u_view_projection", viewProjection);

    shader.setUniform("u_cam_pos", Vector3f(cam_pos.x, cam_pos.y, cam_pos.z), false);

    if ( enableDensityDraw ){ drawDensityPoints( shader ); }
  }

  /* Draw lines from density pt to closest pt */
  {
    auto& user_shad = shader_map["NoLighting"];

    GLShader& shader = *(user_shad.nanogui_shader);
    shader.bind();

    // Prepare the camera projection matrix
    shader.setUniform("u_model", model);
    shader.setUniform("u_view_projection", viewProjection);

    shader.setUniform("u_cam_pos", Vector3f(cam_pos.x, cam_pos.y, cam_pos.z), false);

    if ( enableLinesDraw ) { drawLines( shader ); }
  }

  /* Draw Point Light */
  {
    auto& user_shad = shader_map["PointLight"];

    GLShader& shader = *(user_shad.nanogui_shader);
    shader.bind();

    // Prepare the camera projection matrix
    shader.setUniform("u_model", model);
    shader.setUniform("u_view_projection", viewProjection);

    drawPointLight( shader );
  }


  /* Draw Triangle */
  {
    auto& user_shad = shader_map["Default"];

    GLShader& shader = *(user_shad.nanogui_shader);
    shader.bind();

    // Prepare the camera projection matrix
    shader.setUniform("u_model", model);
    shader.setUniform("u_view_projection", viewProjection);

    shader.setUniform("u_cam_pos", Vector3f(cam_pos.x, cam_pos.y, cam_pos.z), false);

    // drawTriangle( shader );
  }
}

void Clouds::drawBoundingBox( GLShader &shader ) {
  for ( int i = 0 ; i < num_boxes ; i++ ) {
    shader.setUniform( "u_offset", offsets[0] );
    shader.setUniform( "u_color", nanogui::Color( 0.4f, 0.5f, 1.0f, 1.f )  );
    shader.setUniform( "pt_size", 3.0 * pt_size, false );

    shader.uploadAttrib( "in_position", *positions );

    shader.drawArray( GL_POINTS, 0, positions->cols() );
  }
}

void Clouds::drawWorleyPoints( GLShader &shader ) {
  for ( int i = 0 ; i < num_boxes ; i++ ) {
    /* Render worley points */
    shader.setUniform( "u_offset", offsets[i] );
    shader.setUniform( "u_color", nanogui::Color( 1.f, 0.f, 0.f, 1.f ) );
    shader.setUniform( "pt_size", 1.5 * pt_size, false );

    shader.uploadAttrib( "in_position", *worley_pts );

    shader.drawArray( GL_POINTS, 0, worley_pts->cols() );
  }
}

void Clouds::drawDensityPoints( GLShader &shader ) {
  for ( int i = 0 ; i < num_boxes ; i++ ) {
    /* Render density points */
    shader.setUniform( "u_color", Vector4f( 1.f, 1.f, 1.f, 1.f ), false );
    shader.setUniform( "u_offset", offsets[i] );
    shader.setUniform( "pt_size", pt_size, false );

    shader.uploadAttrib( "in_position", *density_pts );
    shader.uploadAttrib( "in_density", *density_vals );

    shader.drawArray( GL_POINTS, 0, density_pts->cols() );
  }
}

void Clouds::drawLines( GLShader &shader ) {
  shader.setUniform("u_color", nanogui::Color( 1.f, 1.f, 1.f, 0.2f ) );

  for ( int i = 0 ; i < num_boxes ; i++ ) {
    shader.setUniform( "u_offset", offsets[i] );
    shader.uploadAttrib( "in_position", *lines );
    shader.drawArray( GL_LINES, 0, lines->cols() );
  }
}

void Clouds::drawPointLight( GLShader &shader ) {
  shader.setUniform( "u_color", nanogui::Color( 1.f, 0.95f, 0.5f, 1.0f ) );
  shader.uploadAttrib( "in_pt_light_pos", pt_light_pos );

  shader.drawArray( GL_POINTS, 0, 1 );
}

void Clouds::drawTriangle(GLShader &shader) {
  MatrixXf positions(3, 3);

  // RED :: +y +x +z
  positions <<  1.0,  0.0, 0.0,
                0.0,  1.0, 0.0,
                0.0,  0.0, 1.0;

  shader.uploadAttrib( "in_position", positions, false );
  shader.setUniform("u_color", nanogui::Color(1.0f, 0.0f, 0.0f, 0.1f), false);
  shader.drawArray( GL_TRIANGLES, 0, 3 );

  // BLUE :: +y -x -z
  positions <<  -1.0,  0.0, 0.0,
                0.0,  1.0, 0.0,
                0.0,  0.0, -1.0;

  shader.uploadAttrib( "in_position", positions, false );
  shader.setUniform("u_color", nanogui::Color(0.0f, 0.0f, 1.0f, 0.1f), false);
  shader.drawArray( GL_TRIANGLES, 0, 3 );

  // GREEN :: +y +x -z
  positions <<  1.0,  0.0, 0.0,
                0.0,  1.0, 0.0,
                0.0,  0.0, -1.0;

  shader.uploadAttrib( "in_position", positions, false );
  shader.setUniform("u_color", nanogui::Color(0.0f, 1.0f, 0.0f, 0.1f), false);
  shader.drawArray( GL_TRIANGLES, 0, 3 );

  // PURPLE :: +y -x +z
  positions <<  -1.0,  0.0, 0.0,
                0.0,  1.0, 0.0,
                0.0,  0.0, 1.0;

  shader.uploadAttrib( "in_position", positions, false );
  shader.setUniform("u_color", nanogui::Color(1.0f, 0.0f, 1.0f, 0.1f), false);
  shader.drawArray( GL_TRIANGLES, 0, 3 );

  // BOTTOM
  positions <<  0.0,  1.0, 0.0,
                0.0,  0.0, 0.0,
               -1.0,  0.0, 1.0;

  shader.uploadAttrib( "in_position", positions, false );
  shader.setUniform("u_color", nanogui::Color(1.0f, 1.0f, 1.0f, 0.1f), false);
  shader.drawArray( GL_TRIANGLES, 0, 3 );

  positions <<  0.0, -1.0, 0.0,
                0.0,  0.0, 0.0,
               -1.0,  0.0, 1.0;

  shader.uploadAttrib( "in_position", positions, false );
  shader.setUniform("u_color", nanogui::Color(1.0f, 1.0f, 1.0f, 0.1f), false);
  shader.drawArray( GL_TRIANGLES, 0, 3 );
}

