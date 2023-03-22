#include <fstream>
#include <sstream>
#include <string>

#define GLFW_INCLUDE_GLU 
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "bvh.hh"
#include "renderer.hh"
#include "main.hh"

struct TriangleBound
{
    TriangleBound(const std::vector<Vec3> &vs, const std::vector<Int3> &fs): vs(vs), fs(fs) {}
    inline Box3 operator() (int fid) const;
    const std::vector<Vec3> &vs;
    const std::vector<Int3> &fs;
};

inline Box3 TriangleBound::operator()(int fid) const
{
    const auto &f = fs[fid];
    return make_aabb<double, 3>(vs[f[0]], vs[f[1]], vs[f[2]]);
}

inline bool is_intersecting(
    const Vec3 &v0,
    const Vec3 &v1,
    const Vec3 &v2,
    const Vec3 &org,
    const Vec3 &dir,
    double &dist,
    bool culling)
{
    constexpr double kEps = std::numeric_limits<double>::epsilon();

    auto v01 = v1 - v0;
    auto v02 = v2 - v0;
    auto pvc = cross(dir, v02); // T
    double det = dot(v01, pvc); // ((P1, V02, V01))

    if (culling && det < 0) return false;
    if (!culling && std::abs(det) < kEps) return false;

    double inv = 1 / det;
    auto tvc = org - v0; // P0 - V0
    double u = dot(tvc, pvc) * inv; // Eq.3
    if (u < 0 || u > 1) return false;

    auto qvc = cross(tvc, v01); // S
    double v = dot(dir, qvc) * inv; // Eq.4
    if (v < 0 || u + v > 1) return false;

    // distance from ray.origin to hit point
    double t = dot(v02, qvc) * inv; // Eq.5

    // update hit distance
    if (t > 0 && dist > t)
    {
        dist = t;
        return true; // ray hit primitive in distance
    }
    else return false; // ray hit primitive out of distance
}

struct TriangleCollide
{
    TriangleCollide(const std::vector<Vec3> &vs, const std::vector<Int3> &fs): vs(vs), fs(fs) {}
    inline bool operator() (int fid, const Vec3 &org, const Vec3 &dir, double &dist) const;
    const std::vector<Vec3> &vs;
    const std::vector<Int3> &fs;
    mutable int fc { -1 };
};

inline bool TriangleCollide::operator() (int fid, const Vec3 &org, const Vec3 &dir, double &dist) const
{
    const auto &f = fs[fid];
    bool hit = is_intersecting(vs[f[0]], vs[f[1]], vs[f[2]], org, dir, dist, true);
    if (hit) fc = fid;
    return hit;
}

static int read_mesh_obj(const char *filename, std::vector<Vec3> &vs, std::vector<Int3> &fs)
{
    constexpr size_t kInf = std::numeric_limits<std::streamsize>::max();

    std::ifstream in(filename, std::ios::in);
    if (!in) { fprintf(stderr, "Cannot open file %s\n", filename); return 1; }

    vs.clear(); fs.clear();

    for (std::string line; std::getline(in, line); )
    {
        std::stringstream ss(line);
        std::string token {};
        ss >> token;

        if (token.compare("v") == 0) // vertex
        {
            Vec3 v {};
            ss >> v[0]; ss >> v[1]; ss >> v[2];
            vs.push_back(v);
        }
        else if (token.compare("f") == 0) // face
        {
            Int3 vtn[4] = { {-1,-1,-1}, {-1,-1,-1}, {-1,-1,-1}, {-1,-1,-1} };
            std::string ids; // v or v/t or v/t/n

            for (int i = 0; ss >> ids; ++i)
            {
                //std::cout<<"ids: "<<ids<<std::endl;
                std::stringstream si(ids);
                std::string id {};

                for (int j = 0; std::getline(si, id, '/'); ++j)
                {
                    //std::cout<<"id: "<<id<<std::endl;
                    std::stringstream sj(id);
                    sj >> vtn[i][j];
                }
            }
            //std::cout<<vtn[0][0]<<", "<<vtn[1][0]<<", "<<vtn[2][0]<<std::endl;

            fs.push_back(Int3 { vtn[0][0], vtn[1][0], vtn[2][0] } - 1);
            if (vtn[3][0] != -1) fs.push_back(Int3 { vtn[0][0], vtn[2][0], vtn[3][0] } - 1);
        }
        else if (token.compare("vt") == 0) // uv
        {
        }
        else if (token.compare("vn") == 0) // normal
        {
        }
    }

    return 0;
}

static void normalize(std::vector<Vec3> &vs)
{
    Box3 b = make_aabb<double, 3>();

    for (const auto &v : vs)
        b = merge(b, make_aabb<double, 3>(v));

    const auto d = max_component(b);
    const auto c = centroid(b);

    for (auto &v : vs)
        v = (v - c) / d * 2.0;
}

static void set_vertex_normal(const std::vector<Vec3> &vs, const std::vector<Int3> &fs, std::vector<Vec3> &ns)
{
    ns.clear(); ns.resize(vs.size(), { 0,0,0 });

    for (const auto &f : fs)
    {
        const auto &v0 = vs[f[0]];
        const auto &v1 = vs[f[1]];
        const auto &v2 = vs[f[2]];
        auto n = normalize(cross(v1 - v0, v2 - v0));
        ns[f[0]] += n;
        ns[f[1]] += n;
        ns[f[2]] += n;
    }

    for (auto &n : ns) n = normalize(n);
}

static int select_mesh_face(
    const Bvh<int, double, 3> &bvh,
    const std::vector<Vec3> &vs,
    const std::vector<Int3> &fs,
    const Vec3 &org,
    const Vec3 &dir)
{
    double dist { 1e10 };
    TriangleCollide collide(vs, fs);
    if (bvh.intersect(collide, org, dir, dist))
        return collide.fc;
    return -1;
}

int main(int argc, const char **argv)
{
    if (argc < 2) return 1;

    std::vector<Vec3> vs {};
    std::vector<Int3> fs {};

    // populate data
    if (read_mesh_obj(argv[1], vs, fs) != 0) return 1;
    normalize(vs);

    std::vector<Vec3> ns {};
    set_vertex_normal(vs, fs, ns);

    // build accel
    Bvh<int, double, 3> bvh;
    TriangleBound bound(vs, fs); {
    //EqualCountsSplit<int, TriangleBound, double, 3> split(bound);
    //MiddlePointSplit<int, TriangleBound, double, 3> split(bound);
    SAHSplit<int, TriangleBound, double, 3> split(bound);
    std::vector<int> fids {}; for (int i=0; i<fs.size(); ++i) fids.push_back(i);
    bvh.build<TriangleBound, decltype(split)>(fids, bound, split, 1); }

    // attributes
    std::vector<bool> selected(fs.size(), false);

    // OpenGL
    glfwSetErrorCallback(error_callback);
    if (!glfwInit()) { exit(EXIT_FAILURE); }

    constexpr int kw { 1200 }, kh { 800 };
    GLFWwindow* window = glfwCreateWindow(kw, kh, "Obj Viewer", NULL, NULL);
    if (!window) { glfwTerminate(); exit(EXIT_FAILURE); }

    glfwSetKeyCallback(window, keyboard_callback);
    glfwSetScrollCallback(window, cursor_scroll_callback);
    glfwSetCursorPosCallback(window, cursor_move_callback);
    glfwSetMouseButtonCallback(window, cursor_press_callback);
    glfwSetFramebufferSizeCallback(window, window_resize_callback);

    glfwMakeContextCurrent(window);
    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    set_perspective_projection(45.0, kw / (float)kh);
    glClearColor(0.17f, 0.17f, 0.41f, 0); // background
    glViewport(0, 0, kw, kh);
    set_light_sources();

    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPushMatrix();
        glLoadIdentity();
        set_camera_transform();
        set_world_transform();
        //render_xyz_axis();
        //render_unit_box();
        //render_mesh_faces(vs, fs);
        render_mesh_faces(vs, fs, ns);
        render_mesh_edges(vs, fs);
        render_selected_faces(vs, fs, selected);
        //for (const auto &node : bvh.nodes()) render_bounding_box(node.b);
        glPopMatrix();
        if (g_cursor_select)
        {
            int width {}, height {}; glfwGetFramebufferSize(window, &width, &height);
            double xpos {}, ypos {}; glfwGetCursorPos(window, &xpos, &ypos);
            Vec3 org {}, dir {}; screen_coords_to_ray(xpos, height - ypos,
                org[0], org[1], org[2], dir[0], dir[1], dir[2]);
            int fid = select_mesh_face(bvh, vs, fs, org, dir);
            if (fid != -1) selected[fid] = !selected[fid];
            g_cursor_select = false;
        }
        glfwPollEvents();
        glfwSwapBuffers(window);
        std::fflush(stdout);
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}