#include "clouds.h"

// Needed to generate stb_image binaries. Should only define in exactly one source file importing stb_image.h.
#define STB_IMAGE_IMPLEMENTATION
#include "misc/stb_image.h"

using namespace nanogui;
using namespace std;

Vector3D load_texture(int frame_idx, GLuint handle, const char* where) {
  Vector3D size_retval;
  
  if (strlen(where) == 0) return size_retval;
  
  glActiveTexture(GL_TEXTURE0 + frame_idx);
  glBindTexture(GL_TEXTURE_2D, handle);
  
  int img_x, img_y, img_n;
  unsigned char* img_data = stbi_load(where, &img_x, &img_y, &img_n, 3);
  size_retval.x = img_x;
  size_retval.y = img_y;
  size_retval.z = img_n;
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_x, img_y, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  stbi_image_free(img_data);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  return size_retval;
}

void load_cubemap(int frame_idx, GLuint handle, const std::vector<std::string>& file_locs) {
  glActiveTexture(GL_TEXTURE0 + frame_idx);
  glBindTexture(GL_TEXTURE_CUBE_MAP, handle);
  for (int side_idx = 0; side_idx < 6; ++side_idx) {
    int img_x, img_y, img_n;
    unsigned char* img_data = stbi_load(file_locs[side_idx].c_str(), &img_x, &img_y, &img_n, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + side_idx, 0, GL_RGB, img_x, img_y, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
    stbi_image_free(img_data);
    std::cout << "Side " << side_idx << " has dimensions " << img_x << ", " << img_y << std::endl;

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  }
}

void Clouds::load_textures() {
  glGenTextures(1, &m_gl_texture_1);
  glGenTextures(1, &m_gl_texture_2);
  glGenTextures(1, &m_gl_texture_3);
  glGenTextures(1, &m_gl_texture_4);
  glGenTextures(1, &m_gl_cubemap_tex);
  
  m_gl_texture_1_size = load_texture(1, m_gl_texture_1, (m_project_root + "/textures/texture_1.png").c_str());
  m_gl_texture_2_size = load_texture(2, m_gl_texture_2, (m_project_root + "/textures/texture_2.png").c_str());
  m_gl_texture_3_size = load_texture(3, m_gl_texture_3, (m_project_root + "/textures/texture_3.png").c_str());
  m_gl_texture_4_size = load_texture(4, m_gl_texture_4, (m_project_root + "/textures/texture_4.png").c_str());
  
  std::cout << "Texture 1 loaded with size: " << m_gl_texture_1_size << std::endl;
  std::cout << "Texture 2 loaded with size: " << m_gl_texture_2_size << std::endl;
  std::cout << "Texture 3 loaded with size: " << m_gl_texture_3_size << std::endl;
  std::cout << "Texture 4 loaded with size: " << m_gl_texture_4_size << std::endl;
  
  std::vector<std::string> cubemap_fnames = {
    m_project_root + "/textures/cube/posx.jpg",
    m_project_root + "/textures/cube/negx.jpg",
    m_project_root + "/textures/cube/posy.jpg",
    m_project_root + "/textures/cube/negy.jpg",
    m_project_root + "/textures/cube/posz.jpg",
    m_project_root + "/textures/cube/negz.jpg"
  };
  
  load_cubemap(5, m_gl_cubemap_tex, cubemap_fnames);
  std::cout << "Loaded cubemap texture" << std::endl;
}

void Clouds::load_shaders() {
  std::set<std::string> shader_folder_contents;
  bool success = FileUtils::list_files_in_directory(m_project_root + "/shaders", shader_folder_contents);
  if (!success) {
    std::cout << "Error: Could not find the shaders folder!" << std::endl;
  }
  
  std::string std_vert_shader = m_project_root + "/shaders/Default.vert";
  
  for (const std::string& shader_fname : shader_folder_contents) {
    std::string file_extension;
    std::string shader_name;
    
    FileUtils::split_filename(shader_fname, shader_name, file_extension);
    
    if (file_extension != "frag") {
      std::cout << "Skipping non-shader file: " << shader_fname << std::endl;
      continue;
    }
    
    std::cout << "Found shader file: " << shader_fname << std::endl;
    
    // Check if there is a proper .vert shader or not for it
    std::string vert_shader = std_vert_shader;
    std::string associated_vert_shader_path = m_project_root + "/shaders/" + shader_name + ".vert";
    if (FileUtils::file_exists(associated_vert_shader_path)) {
      vert_shader = associated_vert_shader_path;
    }
    
    std::shared_ptr<GLShader> nanogui_shader = make_shared<GLShader>();
    nanogui_shader->initFromFiles(shader_name, vert_shader,
                                  m_project_root + "/shaders/" + shader_fname);
    
    // Special filenames are treated a bit differently
    ShaderTypeHint hint;
    if (shader_name == "Wireframe") {
      hint = ShaderTypeHint::WIREFRAME;
      std::cout << "Type: Wireframe" << std::endl;
    }
    
    UserShader user_shader(shader_name, nanogui_shader, hint);
    
    shader_map.emplace( shader_name, user_shader );
    // shader_map[shader_name] = user_shader;
    shaders.push_back(user_shader);
    shaders_combobox_names.push_back(shader_name);
  }
  
  // Assuming that it's there, use "Wireframe" by default
  for (size_t i = 0; i < shaders_combobox_names.size(); ++i) {
    if (shaders_combobox_names[i] == "Wireframe") {
      active_shader_idx = i;
      break;
    }
  }
}

Clouds::Clouds(std::string project_root, Screen *screen)
: m_project_root(project_root) {
  this->screen = screen;
  
  this->load_shaders();
  this->load_textures();

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);

}

Clouds::~Clouds() {
  for (auto shader : shaders) {
    shader.nanogui_shader->free();
  }

  glDeleteTextures(1, &m_gl_texture_1);
  glDeleteTextures(1, &m_gl_texture_2);
  glDeleteTextures(1, &m_gl_texture_3);
  glDeleteTextures(1, &m_gl_texture_4);
  glDeleteTextures(1, &m_gl_cubemap_tex);
}

int Clouds::getFPS() {
  return frames_per_sec;
}

/**
 * Initializes the clouds simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void Clouds::init() {
  // Initialize GUI
  screen->setSize(default_window_size);
  initGUI(screen);

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target

  Vector3D avg_pm_position(0, 0, 0);

  CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2,
                       avg_pm_position.z);
  CGL::Vector3D c_dir(0., 0., 0.);
  canonical_view_distance = 2;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 20.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);

  generatePoints();
}

bool Clouds::isAlive() { return is_alive; }

void Clouds::update( double dt ) { }

void Clouds::updateGUI( double avgFPS ) {
  // Update GUI
  fps_box->setValue( std::ceil( avgFPS ) );
}
/**********************************************************************************
 *
 *                                 Cloud Drawing
 *
 ***********************************************************************************/

/**
 * Generates a set of bounding points (corners) for a 3D box
 * and places them into Clouds::positions.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateBoundingPoints(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( positions != nullptr ) { delete positions; }
  positions =  new MatrixXf( 3, matrixDimension );

  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        Vector3f corner = Vector3f( x, y, z ) * cell_size;
        unsigned long i = z + y * numberOfCells + x * numberOfCells * numberOfCells;
        positions->col( i ) = corner;
      }
    }
  }
}

/**
 * Generates a set of random Worley points for each box in 3D space
 * and places them into Clouds::worley_pts.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateWorleyPoints(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( worley_pts != nullptr ) { delete worley_pts; }
  worley_pts = new MatrixXf( 3, matrixDimension );

  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        Vector3f corner = Vector3f( x, y, z ) * cell_size;

        Vector3f R = Vector3f::Random(3).array().abs();

        unsigned long i = z + y * numberOfCells + x * numberOfCells * numberOfCells;
        Vector3f pt = corner + R * cell_size;
        worley_pts->col( i ) = pt;
      }
    }
  }
}

/**
 * Generates a set of density points and assigns them with a value:
 * For each pixel [density point] -> calculate the distance between
 *                                   that pixel and the closest Worley
 *                                   point [density value]
 * Results are placed into Clouds::density_pts & Clouds::density_vals.
 * This is a helper for Clouds::generatePoints().
 */
void Clouds::generateDensityValues(int numberOfCells) {
  int matrixDimension = pow( numberOfCells, 3 );
  float cell_size = 1. / num_cells;

  if ( density_pts != nullptr ) { delete density_pts; }
  density_pts = new MatrixXf( 3, matrixDimension * matrixDimension );

  if ( density_vals != nullptr ) { delete density_vals; }
  density_vals = new MatrixXf( 3, matrixDimension * matrixDimension );
  float density_cell_size = cell_size / numberOfCells;

  if ( lines != nullptr ) { delete lines; }
  lines = new MatrixXf( 3, 2 * matrixDimension * matrixDimension );

  // for each cell (a cell contains 1 worley point)
  float max_dist = std::numeric_limits<float>::min();
  for ( int x = 0 ; x < numberOfCells ; x++ ) {
    for ( int y = 0 ; y < numberOfCells ; y++ ) {
      for ( int z = 0 ; z < numberOfCells ; z++ ) {
        int i = z + y * numberOfCells + x * numberOfCells * numberOfCells; // cell index
        Vector3f corner = Vector3f( x, y, z ) * cell_size;

        // for each density block inside the cell
        for ( int xx = 0 ; xx < numberOfCells ; xx++ ) {
          for ( int yy = 0 ; yy < numberOfCells ; yy++ ) {
            for ( int zz = 0 ; zz < numberOfCells ; zz++ ) {
              int j = zz + yy * numberOfCells + xx * numberOfCells * numberOfCells; // density cell index
              Vector3f density_corner = corner + Vector3f( xx, yy, zz ) * density_cell_size;
              Vector3f density_center = density_corner + Vector3f( 1., 1., 1. ) * density_cell_size / 2;

              // search 27 cells for closest Worley point
              const Vector3f start = Vector3f( x - 1, y - 1, z - 1 );
              float min_dist = std::numeric_limits<float>::max();
              Vector3f min_pt;

              for ( int sx = start.x() ; sx < start.x() + 3 ; sx++  ) {
                int sxx = (sx < 0 ? numberOfCells + sx : sx) % numberOfCells;
                for ( int sy = start.y() ; sy < start.y() + 3 ; sy++ ) {
                  int syy = (sy < 0 ? numberOfCells + sy : sy) % numberOfCells;
                  for ( int sz = start.z() ; sz < start.z() + 3 ; sz++ ) {
                    int szz = (sz < 0 ? numberOfCells + sz : sz) % numberOfCells;

                    if ( sx != sxx || sy != syy || sz != szz ) {
                      // wrapping

                      // index into cells w/ wrapping
                      int k = szz + syy * numberOfCells + sxx * numberOfCells * numberOfCells;
                      Vector3f v = worley_pts->col( k );
                      float dist = (density_center - v).norm();

                      for ( int tx = 0, offx = -1 ; tx < numberOfCells ; tx++, offx++ ) {
                        for ( int ty = 0 , offy = -1 ; ty < numberOfCells ; ty++, offy++ ) {
                          for ( int tz = 0 , offz = -1 ; tz < numberOfCells ; tz++, offz++ ) {
                            Vector3f V = v + Vector3f( offx, offy, offz );
                            float dist = (density_center - V).norm();

                            if ( dist < min_dist ) {
                              min_pt = V;
                              min_dist = dist;
                            }
                          }
                        }
                      }

                    } else {
                      // not wrapping

                      int k = sz + sy * numberOfCells + sx * numberOfCells * numberOfCells; // index into cells w/ wrapping
                      Vector3f v = worley_pts->col( k );

                      float dist = (density_center - v).norm();

                      if ( dist < min_dist ) {
                        min_pt = v;
                        min_dist = dist;
                      }
                    }

                  }
                }
              }

              int I = i * pow( numberOfCells, 3 ) + j;

              lines->col( 2 * I ) = density_center;
              lines->col( 2 * I + 1 ) = min_pt;

              density_pts->col( I ) = density_center;
              density_vals->col( I ) = Vector3f( min_dist, 0, 0 );

              // Find maximum value for nomalizing
              if ( min_dist > max_dist ) {
                max_dist = min_dist;
              }
            }
          }
        } // end for each density block inside the cell

      }
    }
  } // end for each cell

  *density_vals /= max_dist;
}

void Clouds::generatePoints() {
  generateBoundingPoints(num_cells + 1);
  generateWorleyPoints(num_cells);
  generateDensityValues(num_cells);
}

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
    shader.setUniform( "u_color", nanogui::Color( 0.3f, 0.5f, 1.0f, 1.f )  );
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

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// On initial load, +z faces camera, +x points right, +y points up
// ----------------------------------------------------------------------------

void Clouds::resetCamera() { camera.copy_placement(canonicalCamera); }

Matrix4f Clouds::getProjectionMatrix() {
  Matrix4f perspective;
  perspective.setZero();

  double cam_near = camera.near_clip();
  double cam_far = camera.far_clip();

  double theta = camera.v_fov() * PI / 360;
  double range = cam_far - cam_near;
  double invtan = 1. / tanf(theta);

  perspective(0, 0) = invtan / camera.aspect_ratio();
  perspective(1, 1) = invtan;
  perspective(2, 2) = -(cam_near + cam_far) / range;
  perspective(3, 2) = -1;
  perspective(2, 3) = -2 * cam_near * cam_far / range;
  perspective(3, 3) = 0;

  return perspective;
}

Matrix4f Clouds::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

  CGL::Vector3D c_pos = camera.position();
  CGL::Vector3D c_udir = camera.up_dir();
  CGL::Vector3D c_target = camera.view_point();

  Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
  Vector3f up(c_udir.x, c_udir.y, c_udir.z);
  Vector3f target(c_target.x, c_target.y, c_target.z);

  R.col(2) = (eye - target).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));

  lookAt.topLeftCorner<3, 3>() = R.transpose();
  lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
  lookAt(3, 3) = 1.0f;

  return lookAt;
}

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool Clouds::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x = x;
  mouse_y = y;

  return true;
}

bool Clouds::mouseButtonCallbackEvent(int button, int action, int modifiers) {
  switch (action) {
  case GLFW_PRESS:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = true;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = true;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = true;
      break;
    }
    return true;

  case GLFW_RELEASE:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = false;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = false;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = false;
      break;
    }
    return true;
  }

  return false;
}

void Clouds::mouseMoved(double x, double y) { y = screen_h - y; }

void Clouds::mouseLeftDragged(double x, double y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;

  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void Clouds::mouseRightDragged(double x, double y) {
  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool Clouds::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
      is_alive = false;
      break;
    case ' ':
      resetCamera();
      break;
    case 'p':
    case 'P':
      is_paused = !is_paused;
      break;
    case 'n':
    case 'N':
      if (is_paused) {
        is_paused = false;
        drawContents();
        is_paused = true;
      }
      break;
    }
  }

  return true;
}

bool Clouds::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool Clouds::scrollCallbackEvent(double x, double y) {
  camera.move_forward(y * scroll_rate);
  return true;
}

bool Clouds::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void Clouds::initGUI(Screen *screen) {
  Window *window;
  
  window = new Window(screen, "Cloud Simulation");
  window->setPosition(Vector2i( 15, 15 ));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  new Label(window, "Buttons", "sans-bold");
  {
    Button *b = new Button(window, "Button 1");
    b->setFlags(Button::ToggleButton);
    b->setPushed( button1 );
    b->setFontSize(14);
    b->setChangeCallback( [this](bool state) {  } );

    Button *b1 = new Button(window, "Bounding Points");
    b1->setFlags(Button::ToggleButton);
    b1->setPushed( enableBoundingBoxDraw );
    b1->setFontSize(14);
    b1->setChangeCallback( [&](bool state) { enableBoundingBoxDraw = state; } );

    Button *b2 = new Button(window, "Worley Points");
    b2->setFlags(Button::ToggleButton);
    b2->setPushed( enableWorleyDraw );
    b2->setFontSize(14);
    b2->setChangeCallback( [this](bool state) { enableWorleyDraw = state; } );

    Button *b3 = new Button(window, "Density Points");
    b3->setFlags(Button::ToggleButton);
    b3->setPushed( enableDensityDraw );
    b3->setFontSize(14);
    b3->setChangeCallback( [this](bool state) { enableDensityDraw = state; } );

    Button *b4 = new Button(window, "Lines");
    b4->setFlags(Button::ToggleButton);
    b4->setPushed( enableLinesDraw );
    b4->setFontSize(14);
    b4->setChangeCallback( [this](bool state) { enableLinesDraw = state; } );
  }

  new Label(window, "Parameters", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "param 1 :", "sans-bold");

    FloatBox<double> *fb = new FloatBox<double>(panel);
    fb->setEditable(true);
    fb->setFixedSize(Vector2i(100, 20));
    fb->setFontSize(14);
    fb->setValue( param1 );
    fb->setUnits("units");
    fb->setSpinnable(true);
    fb->setCallback([this](float value) {  });

    new Label(panel, "num boxes :", "sans-bold");

    num_cells_box = new IntBox<int>(panel);
    num_cells_box->setEditable(true);
    num_cells_box->setFixedSize(Vector2i(100, 20));
    num_cells_box->setFontSize(14);
    num_cells_box->setValue( num_boxes );
    num_cells_box->setMinValue( 0 );
    num_cells_box->setUnits("cells");
    num_cells_box->setSpinnable(true);
    num_cells_box->setCallback([this](int value) { num_boxes = value; });

    new Label(panel, "num cells :", "sans-bold");

    num_cells_box = new IntBox<int>(panel);
    num_cells_box->setEditable(true);
    num_cells_box->setFixedSize(Vector2i(100, 20));
    num_cells_box->setFontSize(14);
    num_cells_box->setValue( num_cells );
    num_cells_box->setMinValue( 0 );
    num_cells_box->setUnits("cells");
    num_cells_box->setSpinnable(true);
    num_cells_box->setCallback([this](int value) {
        num_cells = value;
        generatePoints();
    });

    new Label(panel, "pt size :", "sans-bold");

    auto pt_size_box = new FloatBox<float>(panel);
    pt_size_box->setEditable(true);
    pt_size_box->setFixedSize(Vector2i(100, 20));
    pt_size_box->setFontSize(14);
    pt_size_box->setValue( pt_size );
    pt_size_box->setMinValue( 0 );
    pt_size_box->setUnits("px");
    pt_size_box->setSpinnable(true);
    pt_size_box->setCallback([this](float value) {
        pt_size = value;
    });
  }

  new Label(window, "Sliders", "sans-bold");
  {
    Widget *panel = new Widget(window);
    panel->setLayout(
        new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *s = new Slider(panel);
    s->setValue( slider );
    s->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string( slider ));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    s->setCallback([percentage](float value) {
      percentage->setValue(std::to_string(value));
    });
    s->setFinalCallback([&](float value) {
      slider = value;
    });
  }

  new Label(window, "Statistics", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "AVG FPS :", "sans-bold");

    fps_box = new IntBox<int>(panel);
    fps_box->setEditable(false);
    fps_box->setFixedSize(Vector2i(100, 20));
    fps_box->setFontSize(14);
    fps_box->setValue( 0 );
    fps_box->setUnits("fps");
    fps_box->setSpinnable(false);
  }
}
