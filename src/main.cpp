#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include "misc/getopt.h" // getopt for windows
#else
#include <getopt.h>
#include <unistd.h>
#endif
#include <unordered_set>
#include <stdlib.h> // atoi for getopt inputs
#include <chrono>
#include <thread>

#include "CGL/CGL.h"
#include "clouds.h"
#include "json.hpp"
#include "misc/file_utils.h"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

Clouds*     app    = nullptr;
GLFWwindow* window = nullptr;
Screen*     screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Cloud Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n\n", binaryName);
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

bool is_valid_project_root(const std::string& search_path) {
    std::stringstream ss;
    ss << search_path;
    ss << "/";
    ss << "shaders/Default.vert";
    
    return FileUtils::file_exists(ss.str());
}

// Attempt to locate the project root automatically
bool find_project_root(const std::vector<std::string>& search_paths, std::string& retval) {
  for (std::string search_path : search_paths) {
    if (is_valid_project_root(search_path)) {
      retval = search_path;
      return true;
    }
  }
  return false;
}


/********************************************************************************
 *                              Compute Shader
 ********************************************************************************/

// helper to check and display for shader compiler errors
bool check_shader_compile_status(GLuint obj) {
    GLint status;
    glGetShaderiv(obj, GL_COMPILE_STATUS, &status);
    if(status == GL_FALSE) {
        GLint length;
        glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &length);
        std::vector<char> log(length);
        glGetShaderInfoLog(obj, length, &length, &log[0]);
        std::cerr << &log[0];
        return false;
    }
    return true;
}

// helper to check and display for shader linker error
bool check_program_link_status(GLuint obj) {
    GLint status;
    glGetProgramiv(obj, GL_LINK_STATUS, &status);
    if(status == GL_FALSE) {
        GLint length;
        glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &length);
        std::vector<char> log(length);
        glGetProgramInfoLog(obj, length, &length, &log[0]);
        std::cerr << &log[0];
        return false;
    }
    return true;
}

void computeTest() {
    int width = 640;
    int height = 480;

    // shader source code

    // we need these to properly pass the strings
    const char *source;
    int length;

    // straight forward implementation of the nbody kernel
    std::string acceleration_source =
        "#version 430\n"
        "layout(local_size_x=256) in;\n"

        "layout(location = 0) uniform float dt;\n"
        "layout(std430, binding=0) buffer pblock { vec4 positions[]; };\n"
        "layout(std430, binding=1) buffer vblock { vec4 velocities[]; };\n"

        "void main() {\n"
        "   int N = int(gl_NumWorkGroups.x*gl_WorkGroupSize.x);\n"
        "   int index = int(gl_GlobalInvocationID);\n"
        "   vec4 v = 2 * positions[index];\n"

        "   positions[index] = v;\n"
        "   velocities[index] = 4 * velocities[index];\n"
        "}\n";

    // program and shader handles
    GLuint acceleration_program, acceleration_shader;

    // create and compiler vertex shader
    acceleration_shader = glCreateShader(GL_COMPUTE_SHADER);
    source = acceleration_source.c_str();
    length = acceleration_source.size();
    glShaderSource(acceleration_shader, 1, &source, &length);
    glCompileShader(acceleration_shader);
    if(!check_shader_compile_status(acceleration_shader)) {
        cout << "check_shader_compile_status(acceleration_shader)\n";
        // glfwDestroyWindow(window);
        // glfwTerminate();
        // return 1;
    }

    // create program
    acceleration_program = glCreateProgram();

    // attach shaders
    glAttachShader(acceleration_program, acceleration_shader);

    // link the program and check for errors
    glLinkProgram(acceleration_program);
    check_program_link_status(acceleration_program);


    const int particles = 8;

    // randomly place particles in a cube
    std::vector<Vector4f> positionData(particles);
    std::vector<Vector4f> velocityData(particles);
    for(int i = 0;i<particles;++i) {
        // initial position
        positionData[i] = Vector4f( i, i, i, i );
        velocityData[i] = Vector4f( i, i, i, i );
    }

    // generate positions_vbos and vaos
    GLuint vao, positions_vbo, velocities_vbo;

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &positions_vbo);
    glGenBuffers(1, &velocities_vbo);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, velocities_vbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Vector4f)*particles, &velocityData[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, positions_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector4f)*particles, &positionData[0], GL_STATIC_DRAW);

    // set up generic attrib pointers
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4*sizeof(GLfloat), (char*)0 + 0*sizeof(GLfloat));


    // Bind both buffers at once
    const GLuint ssbos[] = {positions_vbo, velocities_vbo};
    glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, 0, 2, ssbos);

    // physical parameters
    float dt = 1.0f/60.0f;

    // setup uniforms
    glUseProgram(acceleration_program);
    glUniform1f(0, dt);

    // we are blending so no depth testing
    glDisable(GL_DEPTH_TEST);
    // enable blending
    glEnable(GL_BLEND);
    //  and set the blend function to result = 1*source + 1*destination
    glBlendFunc(GL_ONE, GL_ONE);

    GLuint query;
    glGenQueries(1, &query);

    bool tiled = false;
    bool space_down = false;

    glUseProgram(acceleration_program);
    glDispatchCompute(particles/256, 1, 1);


    glfwSwapBuffers(window);


    for(int i = 0;i<particles;++i) {
        // initial position
        std::cout << "i: " << i << "\npositionData[i]:\n " << positionData[i] << "\n velocityData[i]:\n " << velocityData[i] << "\n\n";
    }

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &positions_vbo);
    glDeleteBuffers(1, &velocities_vbo);

    glDetachShader(acceleration_program, acceleration_shader);
    glDeleteShader(acceleration_shader);
    glDeleteProgram(acceleration_program);


}






void main_loop() {
  // Main clock
  double prev       = glfwGetTime(); // seconds
  double lag        = 0.;
  double frame_time = 1. / app->getFPS();
  
  // Render clock
  double rend_prev = glfwGetTime(); // seconds
  double avg_fps   = 0.;

  computeTest();

  return;
  while (!glfwWindowShouldClose(window)) {
    double now = glfwGetTime();
    double dt = now - prev;

    lag += dt;

    glfwPollEvents();

    bool updated = false;
    while ( lag >= frame_time ) {
      app->update( dt );

      lag -= frame_time;
      updated = true;
      prev = now;
    }

    if ( updated ) {
      double rend_now = glfwGetTime();
      double rend_dt = rend_now - rend_prev;

      if ( rend_dt > 1 ) {
        app->updateGUI( 1. / dt );
        rend_prev = rend_now;
      }

      /* Begin OpenGL Render */
      glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      {
        app->drawContents();

        // Draw nanogui
        screen->drawWidgets();
        screen->drawContents();
      }
      glfwSwapBuffers(window);
      /* End OpenGL Render */

      avg_fps += 1. / dt;
      avg_fps /= 2.;
    } else {
      double ms = 1000 * ( frame_time - dt );
      std::this_thread::sleep_for( std::chrono::milliseconds( (int) ms ) / 2. );
    }

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }
}

int main( int argc, char **argv ) {
  std::vector<std::string> search_paths = {
    ".",
    "..",
    "../..",
    "../../.."
  };
  std::string project_root;
  bool found_project_root = find_project_root(search_paths, project_root);
  
  std::string file_to_load_from;
  bool file_specified = false;

  int opt;
  while ( (opt = getopt (argc, argv, "a:")) != -1 ) {
    switch ( opt ) {
      case 'a': {
        std::cout << "got option -a\n";
        break;
      }
      default: {
        usageError(argv[0]);
        break;
      }
    }
  }
  
  if (!found_project_root) {
    std::cout << "Error: Could not find required file \"shaders/Default.vert\" anywhere!" << std::endl;
    return -1;
  } else {
    std::cout << "Loading files starting from: " << project_root << std::endl;
  }

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the CloudSimulator object
  app = new Clouds(project_root, screen);
  app->init();

  // Call this after all the widgets have been defined
  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window
  setGLFWCallbacks();

  main_loop();

  return 0;
}
