# NBVH
![CMake](https://github.com/IngInx747/nbvh/actions/workflows/.github/workflows/cmake.yml/badge.svg)

`NBVH` is a short of N-dimensional Bounding Volume Hierarchy.

## Setup BVH

Suppose your data type is defined as:

```cpp
using Primitive = /* point, triangle, sphere, etc. */;
using Iter = std::vector<Primitive>::iterator;
```

Define the dimension of bounding box:

```cpp
using Box = Aabb<T, N>;
using Bvh = Bvh<Box>;
```

Tell BVH how to create the bounding box per primitive:

```cpp
struct Bound
{
  Box operator() (const Primitive&)
  { /* build the bounding box of the primitive */ }

  /* other necessary attributes */
};
```

Assign a splitting method with BVH. There are 3 built-in methods(Middle-point, Equal counts and SAH).

```cpp
Bound bound(/* some initializations */);
SAHSplit<Bound, Box, Iter> split(bound);
```

Setup and build the BVH on the given dataset:

```cpp
Bvh bvh ();
std::vector<Primitive> data(/* populated */);
bvh.build(bound, split, data.begin(), data.end());
```

If memory is limited or building time is constrained, use a coarser setting:

```cpp
// stop splitting if #primitives per node is less than the threshold
bvh.build(bound, split, data.begin(), data.begin(), data.end(), 100);
```

## Spatial query

Setup your predicate:

```cpp
struct Predicate
{
  bool operator() (const Box&)
  { /* rough query: check if your searching range hit any box(faster) */ }

  bool operator() (const Primitive&)
  { /* fine query: check if your searching range hit any primitive(slower) */ }

  /* you would like to store the results here */
};
```

Query primitives by the predicate:

```cpp
Predicate pred(/* some initializations */);

if (query(bvh, pred, data.begin()))
{ /* do something */ }
```

## Ray-trace

Setup your ray collision detector:

```cpp
using Vec3 = VectorN<T, 3>;

struct Collide
{
  bool operator() (const Box&, const Vec3 &org, const Vec3 &dir, T &dist)
  { /* do ray-box collision test */ }

  bool operator() (const Primitive&, const Vec3 &org, const Vec3 &dir, T &dist)
  { /* do ray-primitive collision test */ }

  /* you would like to store the results here */
};
```

Trace the ray thru your scene:

```cpp
Collide collide(/* some initializations */);
Vec3 org, dir;
T dist = +inf;

if (intersect(bvh, collide, org, dir, dist, data.begin()))
{ /* do something */ }
```
