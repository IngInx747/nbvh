#ifndef OBJ_VIEWER_GL_HH
#define OBJ_VIEWER_GL_HH

#include <vector>
#include "main.hh"

struct GLFWwindow;

extern bool g_cursor_select;

void set_light_sources();

void render_xyz_axis();

void render_unit_box();

void render_bounding_box(const Box3&);

void render_mesh_faces(const std::vector<Vec3> &vs, const std::vector<Int3> &fs, const std::vector<Vec3> &ns);

void render_mesh_faces(const std::vector<Vec3> &vs, const std::vector<Int3> &fs);

void render_mesh_edges(const std::vector<Vec3> &vs, const std::vector<Int3> &fs);

void render_selected_faces(const std::vector<Vec3> &vs, const std::vector<Int3> &fs, const std::vector<bool>&);

void set_perspective_projection(double fov, double ratio);

void set_camera_transform();

void set_world_transform();

void screen_coords_to_ray(double x, double y, double &ox, double &oy, double &oz, double &dx, double &dy, double &dz);

void error_callback(int err, const char* errmsg);

void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods);

void cursor_move_callback(GLFWwindow *window, double xpos, double ypos);

void cursor_press_callback(GLFWwindow *window, int button, int action, int mods);

void cursor_scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void window_resize_callback(GLFWwindow *window, int width, int height);

#endif