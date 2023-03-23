#include "renderer.hh"
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLFW_INCLUDE_GLU 
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "aabb.hh"
#include "arc_ball.hh"

using glm::vec3;
using glm::vec2;
using glm::mat4;
using glm::quat;
using glm::dvec3;
using glm::dvec2;
using glm::dmat4;
using glm::dquat;

bool g_cursor_select { false };
static ArcBall g_arcball {}; // arc ball
static quat g_rotation { 1, 0, 0, 0 }; // rotation quaternion and 
static vec3 g_translate { 0, 0, 0 };  // translation of the object

static void toggle_gl_state_shade()
{
    GLint state; glGetIntegerv(GL_SHADE_MODEL, &state);
    glShadeModel(state == GL_FLAT ? GL_SMOOTH : GL_FLAT);
}

static void toggle_gl_state_light()
{
    GLboolean state; glGetBooleanv(GL_LIGHTING, &state);
    if (!state) glEnable(GL_LIGHTING);
    else glDisable(GL_LIGHTING);
}

void set_light_sources()
{
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    const GLfloat lightOnePosition[4] = { 0, 0, 1, 0 };
    const GLfloat lightTwoPosition[4] = { 0, 0, -1, 0 };
    const GLfloat lightOneColor[] = { 1, 1, 1, 1.f };
    const GLfloat globalAmb[] = { 0.1f, 0.1f, 0.1f, 1 };
    const GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightOneColor);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);

    const GLfloat mat_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    const GLfloat mat_diffuse[] = { 0.01f, 0.01f, 0.01f, 1.0f };
    const GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    const GLfloat mat_shininess[] = { 32 };
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0f);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    glEnable(GL_LIGHTING);
}

void render_xyz_axis()
{
    static vec3 _xyz[] = {
        { 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };
    static GLuint _idx[] = {
        0, 1, // nx
        0, 2, // ny
        0, 3, // nz
    };
    GLboolean light; glGetBooleanv(GL_LIGHTING, &light); glDisable(GL_LIGHTING);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof(vec3), (const GLvoid*)_xyz);
    glLineWidth(5.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, &_idx[0]);
    glColor3f(0.0f, 1.0f, 0.0f);
    glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, &_idx[2]);
    glColor3f(0.0f, 0.0f, 1.0f);
    glDrawElements(GL_LINES, 2, GL_UNSIGNED_INT, &_idx[4]);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (light) glEnable(GL_LIGHTING);
}

void render_unit_box()
{
    static vec3 _xyz[] = {       // i(zyx)
        { -1.0f, -1.0f, -1.0f }, // 0(000)
        {  1.0f, -1.0f, -1.0f }, // 1(001)
        { -1.0f,  1.0f, -1.0f }, // 2(010)
        {  1.0f,  1.0f, -1.0f }, // 3(011)
        { -1.0f, -1.0f,  1.0f }, // 4(100)
        {  1.0f, -1.0f,  1.0f }, // 5(101)
        { -1.0f,  1.0f,  1.0f }, // 6(110)
        {  1.0f,  1.0f,  1.0f }, // 7(111)
    };
    static GLuint _idx[] = {
        0, 1, 2, 3, 4, 5, 6, 7, // nx
        0, 2, 1, 3, 4, 6, 5, 7, // ny
        0, 4, 1, 5, 2, 6, 3, 7, // nz
    };
    GLboolean light; glGetBooleanv(GL_LIGHTING, &light); glDisable(GL_LIGHTING);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof(vec3), (const GLvoid*)_xyz);
    glLineWidth(1.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, _idx);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (light) glEnable(GL_LIGHTING);
}

void render_bounding_box(const Box3 &bbox)
{
    const Vec3 &a = bbox.p[0];
    const Vec3 &b = bbox.p[1];
    dvec3 _xyz[] = {
        { a[0], a[1], a[2] }, // 0(000)
        { b[0], a[1], a[2] }, // 1(001)
        { a[0], b[1], a[2] }, // 2(010)
        { b[0], b[1], a[2] }, // 3(011)
        { a[0], a[1], b[2] }, // 4(100)
        { b[0], a[1], b[2] }, // 5(101)
        { a[0], b[1], b[2] }, // 6(110)
        { b[0], b[1], b[2] }, // 7(111)
    };
    static GLuint _idx[] = {
        0, 1, 2, 3, 4, 5, 6, 7, // nx
        0, 2, 1, 3, 4, 6, 5, 7, // ny
        0, 4, 1, 5, 2, 6, 3, 7, // nz
    };
    GLboolean light; glGetBooleanv(GL_LIGHTING, &light); glDisable(GL_LIGHTING);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_DOUBLE, sizeof(dvec3), (const GLvoid*)_xyz);
    glLineWidth(1.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, _idx);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (light) glEnable(GL_LIGHTING);
}

void render_mesh_faces(const std::vector<Vec3> &vs, const std::vector<Int3> &fs, const std::vector<Vec3> &ns)
{
    glEnable(GL_POLYGON_OFFSET_FILL);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glVertexPointer(3, GL_DOUBLE, sizeof(Vec3), (const GLvoid*)vs.data());
    glNormalPointer(GL_DOUBLE, sizeof(Vec3), (const GLvoid*)ns.data());
    glLineWidth(1.0f);
    glColor3ub(220, 220, 220);
    glPolygonMode(GL_FRONT, GL_TRIANGLES);
    glPolygonOffset(1, 1);
    glDrawElements(GL_TRIANGLES, (int)fs.size() * 3, GL_UNSIGNED_INT, (const GLvoid*)fs.data());
    glPolygonMode(GL_FRONT, GL_FILL);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_POLYGON_OFFSET_FILL);
}

void render_mesh_faces(const std::vector<Vec3> &vs, const std::vector<Int3> &fs)
{
    GLboolean light; glGetBooleanv(GL_LIGHTING, &light); glDisable(GL_LIGHTING);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_DOUBLE, sizeof(Vec3), (const GLvoid*)vs.data());
    glLineWidth(1.0f);
    glColor3ub(220, 220, 220);
    glPolygonMode(GL_FRONT, GL_TRIANGLES);
    glPolygonOffset(1, 1);
    glDrawElements(GL_TRIANGLES, (int)fs.size() * 3, GL_UNSIGNED_INT, (const GLvoid*)fs.data());
    glPolygonMode(GL_FRONT, GL_FILL);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_POLYGON_OFFSET_FILL);
    if (light) glEnable(GL_LIGHTING);
}

void render_mesh_edges(const std::vector<Vec3> &vs, const std::vector<Int3> &fs)
{
    GLboolean light; glGetBooleanv(GL_LIGHTING, &light); glDisable(GL_LIGHTING);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_DOUBLE, sizeof(Vec3), (const GLvoid*)vs.data());
    glLineWidth(1.0f);
    glColor3f(0, 0, 0);
    glPolygonMode(GL_FRONT, GL_LINE);
    glDrawElements(GL_TRIANGLES, (int)fs.size() * 3, GL_UNSIGNED_INT, (const GLvoid*)fs.data());
    glPolygonMode(GL_FRONT, GL_FILL);
    glDisableClientState(GL_VERTEX_ARRAY);
    if (light) glEnable(GL_LIGHTING);
}

void render_selected_faces(const std::vector<Vec3> &vs, const std::vector<Int3> &fs, const std::vector<bool> &selected)
{
    GLboolean light; glGetBooleanv(GL_LIGHTING, &light); glDisable(GL_LIGHTING);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(0.5f, 0.5);
    glLineWidth(1.0f);
    glColor3f(1, 1, 0);
    glBegin(GL_TRIANGLES);
    for (int fid = 0; fid < fs.size(); ++fid)
    {
        if (selected[fid])
        {
            for (int i = 0; i < 3; ++i)
            {
                const auto &f = fs[fid];
                const auto &p = vs[f[i]];
                glVertex3d(p[0], p[1], p[2]);
            }
        }
    }
    glEnd();
    glDisable(GL_POLYGON_OFFSET_FILL);
    if (light) glEnable(GL_LIGHTING);
}

void set_perspective_projection(double fov, double ratio)
{
    glMatrixMode(GL_PROJECTION); // switch to proj matrix stack
    glLoadIdentity();
    gluPerspective(
        fov,    // field of view in degrees
        ratio,  // aspect ratio
        0.001,  // Z near
        100.0   // Z far
    );
    glMatrixMode(GL_MODELVIEW); // switch back to model matrix stack
}

void set_camera_transform()
{
    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
}

void set_world_transform()
{
    mat4 rotation = glm::toMat4(g_rotation);
    glTranslatef(g_translate[0], g_translate[1], g_translate[2]); // global transform
    glMultMatrixf(glm::value_ptr(rotation));
}

void screen_coords_to_ray(double x, double y, double &ox, double &oy, double &oz, double &dx, double &dy, double &dz)
{
    int viewport[4];
    double modelViewMatrix[16];
    double projectionMatrix[16];
    double nearPlaneLocation[3];
    double farPlaneLocation[3];

    // Get transformation matrix in OpenGL
    glPushMatrix();
    glLoadIdentity();
    set_camera_transform();
    set_world_transform();
    glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);
    glPopMatrix();

    // Un-project from 2D screen point to real 3D position
    gluUnProject(x, y, 0.0, modelViewMatrix, projectionMatrix,
        viewport, &nearPlaneLocation[0], &nearPlaneLocation[1],
        &nearPlaneLocation[2]);

    gluUnProject(x, y, 1.0, modelViewMatrix, projectionMatrix,
        viewport, &farPlaneLocation[0], &farPlaneLocation[1],
        &farPlaneLocation[2]);

    ox = nearPlaneLocation[0];
    oy = nearPlaneLocation[1];
    oz = nearPlaneLocation[2];

    dx = (farPlaneLocation[0] - nearPlaneLocation[0]);
    dy = (farPlaneLocation[1] - nearPlaneLocation[1]);
    dz = (farPlaneLocation[2] - nearPlaneLocation[2]);
}

void error_callback(int err, const char* errmsg)
{
    fprintf(stderr, "error(%d): %s\n", err, errmsg);
}

void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_S && action == GLFW_PRESS)
    {
        toggle_gl_state_shade();
    }
    if (key == GLFW_KEY_L && action == GLFW_PRESS)
    {
        toggle_gl_state_light();
    }
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

void cursor_move_callback(GLFWwindow *window, double xpos, double ypos)
{
    static dvec2 _xy {}; // last cursor position

    int width {}, height {};
    glfwGetFramebufferSize(window, &width, &height);

    double scale = 10.0 / height;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) // rotation
    {
        quat rot = g_arcball.update_quat((float)xpos - width * 0.5f, height * 0.5f - (float)ypos);
        g_rotation = rot * g_rotation;
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) // zoom in/out
    {
        vec3 trans = glm::vec3(0, 0, (_xy.y - ypos) * scale);
        g_translate = g_translate + trans;
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) // translate
    {
        vec3 trans = glm::vec3(scale * (xpos - _xy.x), scale * (_xy.y - ypos), 0);
        g_translate = g_translate + trans;
    }

    _xy = { xpos, ypos };
}

void cursor_press_callback(GLFWwindow *window, int button, int action, int mods)
{
    static dvec2 _xy {}; // last clicked position

    int width {}, height {};
    double xpos {}, ypos {};

    glfwGetFramebufferSize(window, &width, &height);
    glfwGetCursorPos(window, &xpos, &ypos);

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        g_arcball.set(std::max(width, height) / 2.f, (float)xpos - width / 2.f, height / 2.f - (float)ypos);
        _xy = { xpos, ypos };
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
    {
        if ((int)_xy.x == (int)xpos && (int)_xy.y == (int)ypos) // TODO: set tol
            g_cursor_select = true;
    }
}

void cursor_scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{}

void window_resize_callback(GLFWwindow *window, int width, int height)
{
    set_perspective_projection(45.0, width / (float)height);
    glViewport(0, 0, width, height); // Set Viewport
}
