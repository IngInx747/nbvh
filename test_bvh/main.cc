#include "bvh.hh"

using Vec3 = VectorN<double, 3>;
using Int3 = VectorN<int, 3>;
using Box3 = Aabb<double, 3>;

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

static void set_obj_box(std::vector<Vec3> &vs, std::vector<Int3> &fs)
{
    vs.resize(8); fs.resize(12);
    vs[0] = { -1, -1, -1 };
    vs[1] = {  1, -1, -1 };
    vs[2] = { -1,  1, -1 };
    vs[3] = {  1,  1, -1 };
    vs[4] = { -1, -1,  1 };
    vs[5] = {  1, -1,  1 };
    vs[6] = { -1,  1,  1 };
    vs[7] = {  1,  1,  1 };
    fs[0] = Int3 { 1, 2, 6 } - 1; fs[1] = Int3 { 1, 6, 5 } - 1;
    fs[2] = Int3 { 2, 4, 8 } - 1; fs[3] = Int3 { 2, 8, 6 } - 1;
    fs[4] = Int3 { 4, 3, 7 } - 1; fs[5] = Int3 { 4, 7, 8 } - 1;
    fs[6] = Int3 { 3, 1, 5 } - 1; fs[7] = Int3 { 3, 5, 7 } - 1;
    fs[8] = Int3 { 1, 3, 4 } - 1; fs[9] = Int3 { 1, 4, 2 } - 1;
    fs[10]= Int3 { 5, 6, 8 } - 1; fs[11]= Int3 { 5, 8, 7 } - 1;
}

int main(int argc, const char **argv)
{
    std::vector<Vec3> vs {};
    std::vector<Int3> fs {};

    // populate data
    set_obj_box(vs, fs);

    // build accel
    Bvh<int, double, 3> bvh;
    TriangleBound bound(vs, fs); {
    //EqualCountsSplit<int, TriangleBound, double, 3> split(bound);
    //MiddlePointSplit<int, TriangleBound, double, 3> split(bound);
    SAHSplit<int, TriangleBound, double> split(bound);
    std::vector<int> fids {}; for (int i=0; i<fs.size(); ++i) fids.push_back(i);
    bvh.build<TriangleBound, decltype(split)>(fids, bound, split, 1); }

    TriangleCollide collide(vs, fs);
    double dist { 1e10 };
    bvh.intersect(collide, { -2, 0, 0 }, { 1, 0, 0 }, dist);
    const auto &f = fs[collide.fc];
    std::cout << collide.fc << ": [" << vs[f[0]] << ", " << vs[f[1]] << ", " << vs[f[2]] << "], d = " << dist << std::endl;

    return 0;
}