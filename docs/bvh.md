# Bvh

## When to use BVH

BVH can accelerate spatial search or query in the time complexity of O(log(n)).
It should be used when the dataset is too huge for any brute-force operations,
and remains static that means no data insertion or change in the future.

## How to use BVH

### Setup BVH

Define the data type and dimension of your bounding box.

```cpp
using Primitive = /* point, triangle, sphere, etc. */;
using BvhN = Bvh<Primitive, T, N>;
using BoxN = Aabb<T, N>;
```

Tell BVH how to create the bounding box per primitive.

```cpp
struct MyBoundMethod
{
    inline BoxN operator() (const Primitive&) const
    { /* build the bounding box of the primitive */ }

    /* other necessary attributes */
};
```

Select a box-splitting method for BVH. There are provided with 3 built-in methods(Middle-point, Equal counts and SAH).

```cpp
MyBoundMethod bound(/* some initializations */);
EqualCountsSplit<Primitive, decltype(bound), T, N> split(bound);
```

Setup and build the BVH on the given dataset.

```cpp
BvhN bvh();
std::vector<Primitive> data(/* populated */);
bvh.build(data.begin(), data.end(), bound, split, 1);
```

If memory is a concern and time of building is considerable, try using coarser settings.

```cpp
const int threshold = 100; // stop splitting if primitives per node are fewer than this number
bvh.build(data.begin(), data.end(), bound, split, threshold);
```

### Spatial search

Setup your searching range.

```cpp
struct MyQueryRange
{
    bool operator() (const BvhN&) const
    { /* rough query: check if your searching range hit any box(faster) */ }

    bool operator() (const Primitive&)
    { /* fine query: check if your searching range hit any primitive(slower) */ }

    /* you would like to store the results here */
};
```

Search BVH within the range.

```cpp
MyQueryRange range(/* some initializations */);

if (bvh.search(range))
{ /* do something */ }
```

### Ray-primitive collision

Setup your collision testing method.

```cpp
using Vec3 = VectorN<T, 3>;

struct MyCollidingTest
{
    inline bool operator() (const Primitive&, const Vec3 &org, const Vec3 &dir, T &dist) const
    { /* do ray-primitive collision test */ }

    /* you would like to store the results here */
};
```

Do collision test in the BVH, instead of against all the primitives.

```cpp
MyCollidingTest collide(/* some initializations */);
Vec3 org, dir;
T dist = +inf;

if (bvh.intersect(collide, org, dir, dist))
{ /* do something */ }
```
