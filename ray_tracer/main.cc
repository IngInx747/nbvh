#include <fstream>
#include <sstream>
#include <string>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "bvh.hh"

using Vec3 = VectorN<double, 3>;
using Vec2 = VectorN<double, 2>;
using Int3 = VectorN<int, 3>;
using Box3 = Aabb<double, 3>;
using RGBA = VectorN<unsigned char, 4>;

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
        v = (v - c) / d;
}

inline RGBA to_rgb(const Vec3& p)
{
    const auto v = min(max(p, { 0,0,0 }), { 1,1,1 });
    unsigned char r = (unsigned char)(v[0] * 255.);
    unsigned char g = (unsigned char)(v[1] * 255.);
    unsigned char b = (unsigned char)(v[2] * 255.);
    return { r, g, b, 255 };
}

static void save_image_png(const std::vector<Vec3>& pixels, const int w, const int h, const char* filename)
{
    std::vector<RGBA> result(w * h);

    for (int i = 0; i < h; ++i)
        for (int j = 0; j < w; ++j)
            result[i*w + j] = to_rgb(pixels[i*w + j]);

    stbi_flip_vertically_on_write(1);
    stbi_write_png(filename, w, h, sizeof(RGBA), (void*)result.data(), w * sizeof(RGBA));
}

int main(int argc, const char **argv)
{
    if (argc < 2) return 1;

    std::vector<Vec3> vs {};
    std::vector<Int3> fs {};

    // populate data
    if (read_mesh_obj(argv[1], vs, fs) != 0) return 1;
    normalize(vs);

    // build accel
    Bvh<int, double, 3> bvh;
    TriangleBound bound(vs, fs); {
    //EqualCountsSplit<int, TriangleBound, double, 3> split(bound);
    //MiddlePointSplit<int, TriangleBound, double, 3> split(bound);
    SAHSplit<int, TriangleBound, double> split(bound);
    std::vector<int> fids {}; for (int i=0; i<fs.size(); ++i) fids.push_back(i);
    bvh.build<TriangleBound, decltype(split)>(fids, bound, split, 1); }

    const int w { 800 }, h { 800 };
    const int asp = std::max(w, h);
    const double kFov = 1.79; // cot(22.5)
    std::vector<Vec3> buf(w * h);
    Vec3 org { 0, 0, 1 };

    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            double dist { 1e10 };
            TriangleCollide collide(vs, fs);
            Vec2 dxy { ((j-w/2) + 0.5) / asp, ((i-h/2) + 0.5) / asp }; // [-0.5, 0.5]
            Vec3 dir { dxy[0]*2, dxy[1]*2, -kFov }; dir = normalize(dir);
#if 1
            if (bvh.intersect(collide, org, dir, dist))
#else
            for (int fid = 0; fid < fs.size(); ++fid)
                collide(fid, org, dir, dist);
            if (collide.fc != -1)
#endif
            {
                const auto &f = fs[collide.fc];
                const auto &v0 = vs[f[0]];
                const auto &v1 = vs[f[1]];
                const auto &v2 = vs[f[2]];
                auto n = normalize(cross(v1 - v0, v2 - v0));
                //buf[i*w + j] = { 1,1,1 };
                //buf[i*w + j] = ((v0 + v1 + v2) / 3. + 1.) * .5;
                buf[i*w + j] = (n + 1.) * .5;
            }
        }
    }

    save_image_png(buf, w, h, "output.png");

    return 0;
}