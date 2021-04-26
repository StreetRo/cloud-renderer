#ifndef CGL_CLOUDS_SIMULATOR_H
#define CGL_CLOUDS_SIMULATOR_H

#include <nanogui/nanogui.h>
#include <CGL/vector3D.h>
#include <glad/glad.h>
#include <Eigen/Dense>
#include <nanogui/nanogui.h>

#include "misc/camera_info.h"
#include "misc/file_utils.h"
#include "camera.h"

#include <unordered_map>
#include <cmath>
#include <memory>

using namespace CGL;
using namespace nanogui;

enum ShaderTypeHint { WIREFRAME = 0 };

struct UserShader {
  UserShader(std::string display_name, std::shared_ptr<GLShader> nanogui_shader, ShaderTypeHint type_hint)
  : display_name(display_name)
  , nanogui_shader(nanogui_shader)
  , type_hint(type_hint) {
  }

  UserShader() {}
  
  std::shared_ptr<GLShader> nanogui_shader;
  std::string display_name;
  ShaderTypeHint type_hint;
  
};

class Clouds {
public:
  Clouds(std::string project_root, Screen *screen);
  ~Clouds();

  void init();

  virtual bool isAlive();
  void update( double );
  void updateGUI( double );
  virtual void drawContents();

  // Screen events
  virtual bool cursorPosCallbackEvent(double x, double y);
  virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
  virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
  virtual bool dropCallbackEvent(int count, const char **filenames);
  virtual bool scrollCallbackEvent(double x, double y);
  virtual bool resizeCallbackEvent(int width, int height);

  int getFPS();

private:
  virtual void initGUI(Screen *screen);
  void drawTriangle( GLShader& );
  void drawBoundingPoints( GLShader& );
  void drawBoundingBoxLines( GLShader& );
  void drawBoundingBoxSurface( GLShader& );
  void drawWorleyPoints( GLShader& );
  void drawDensityPoints( GLShader& );
  void drawLines( GLShader& );
  void drawPointLight( GLShader& );
  
  void load_shaders();
  void load_textures();

  // Object Init
  void generatePoints();
  void generateBoundingPoints(int numberOfCells);
  void generateWorleyPoints(int numberOfCells);
  void generateDensityValues(int numberOfCells);
  void generateBoundingBox();

  // File management
  std::string m_project_root;

  // Camera methods
  virtual void resetCamera();
  virtual Matrix4f getProjectionMatrix();
  virtual Matrix4f getViewMatrix();

  // GUI params
  bool button1 = true;
  float param1 = 0.0;
  long slider = 0;
  IntBox<int>* fps_box;
  int num_boxes = 1;

  bool enableBoundingPointsDraw = true;
  bool enableWorleyDraw = true;
  bool enableDensityDraw = true;
  bool enableLinesDraw = true;

  // TODO: refactor this in draw loops
  const std::vector<Vector3f> offsets = {
      // centre
      Vector3f(0.f,0.f,0.f),
      // front face
      Vector3f(0.f,0.f,1.f),
      Vector3f(-1.f,1.f,1.f),
      Vector3f(-1.f,0.f,1.f),
      Vector3f(-1.f,-1.f,1.f),
      Vector3f(0.f,1.f,1.f),
      Vector3f(0.f,-1.f,1.f),
      Vector3f(1.f,1.f,1.f),
      Vector3f(1.f,0.f,1.f),
      Vector3f(1.f,-1.f,1.f),
      // back face
      Vector3f(0.f,0.f,-1.f),
      Vector3f(-1.f,1.f,-1.f),
      Vector3f(-1.f,0.f,-1.f),
      Vector3f(-1.f,-1.f,-1.f),
      Vector3f(0.f,1.f,-1.f),
      Vector3f(0.f,-1.f,-1.f),
      Vector3f(1.f,1.f,-1.f),
      Vector3f(1.f,0.f,-1.f),
      Vector3f(1.f,-1.f,-1.f),
      // ring around centre
      Vector3f(-1.f,1.f,0.f),
      Vector3f(-1.f,0.f,0.f),
      Vector3f(-1.f,-1.f,0.f),
      Vector3f(0.f,1.f,0.f),
      Vector3f(0.f,-1.f,0.f),
      Vector3f(1.f,1.f,0.f),
      Vector3f(1.f,0.f,0.f),
      Vector3f(1.f,-1.f,0.f)
  };

  // Local Persistent Objects
  int n_pts = 5000000;
  int num_cells = 2;
  float pt_size = 10;
  MatrixXf* positions = nullptr;
  MatrixXf* density_pts = nullptr;
  MatrixXf* density_vals = nullptr;
  MatrixXf* worley_pts = nullptr;
  IntBox<int>* num_cells_box;

  MatrixXf* lines = nullptr;

  Vector4f pt_light_pos = Vector4f( 0, 2, -2, 1 );

  // Bounding Box
  MatrixXf bbox_pts = MatrixXf( 3, 24 );
  MatrixXf bbox_tris = MatrixXf( 3, 36 );
  Vector3f bbox_min = Vector3f( 0.f, 0.f, 0.f );
  Vector3f bbox_max = Vector3f( 0.5f, 0.5f, 0.5f );

  // Default simulation values
  int frames_per_sec = 90;
  int simulation_steps = 30;

  CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);

  // OpenGL attributes
  int active_shader_idx = 0;

  std::unordered_map<std::string, UserShader> shader_map;
  std::vector<UserShader> shaders;
  std::vector<std::string> shaders_combobox_names;
  
  // OpenGL textures
  Vector3D m_gl_texture_1_size;
  Vector3D m_gl_texture_2_size;
  Vector3D m_gl_texture_3_size;
  Vector3D m_gl_texture_4_size;
  GLuint m_gl_texture_1;
  GLuint m_gl_texture_2;
  GLuint m_gl_texture_3;
  GLuint m_gl_texture_4;
  GLuint m_gl_cubemap_tex;
  
  // OpenGL customizable inputs
  double m_normal_scaling = 2.0;
  double m_height_scaling = 0.1;

  // Camera attributes
  CGL::Camera camera;
  CGL::Camera canonicalCamera;

  double view_distance;
  double canonical_view_distance;
  double min_view_distance;
  double max_view_distance;

  double scroll_rate;

  // Screen methods
  Screen *screen;
  void mouseLeftDragged(double x, double y);
  void mouseRightDragged(double x, double y);
  void mouseMoved(double x, double y);

  // Mouse flags
  bool left_down = false;
  bool right_down = false;
  bool middle_down = false;

  // Keyboard flags
  bool ctrl_down = false;

  // Simulation flags
  bool is_paused = true;

  // Screen attributes
  int mouse_x;
  int mouse_y;

  int screen_w;
  int screen_h;

  bool is_alive = true;

  Vector2i default_window_size = Vector2i(800, 600);
};

#endif // CGL_CLOUDS_SIM_H
