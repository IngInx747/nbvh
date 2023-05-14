// ======================================================================== //
// Copyright (c) 2022 Ingram Inxent                                         //
//                                                                          //
// Permission is hereby granted, free of charge, to any person obtaining    //
// a copy of this software and associated documentation files (the          //
// "Software"), to deal in the Software without restriction, including      //
// without limitation the rights to use, copy, modify, merge, publish,      //
// distribute, sublicense, and/or sell copies of the Software, and to       //
// permit persons to whom the Software is furnished to do so, subject to    //
// the following conditions:                                                //
//                                                                          //
// The above copyright notice and this permission notice shall be           //
// included in all copies or substantial portions of the Software.          //
//                                                                          //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,          //
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF       //
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                    //
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE   //
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION   //
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION    //
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.          //
// ======================================================================== //

#ifndef AXIS_ALIGNED_BOUNDING_BOX_IMPL_HH
#define AXIS_ALIGNED_BOUNDING_BOX_IMPL_HH

#include "aabb.hh"

////////////////////////////////////////////////////////////////
/// AABB impl utilities
////////////////////////////////////////////////////////////////

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline bool operator<(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_dot<T, N, bool>(a, b, [] (bool r, T x, T y) { return r && x < y; }, true, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline bool operator<=(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_dot<T, N, bool>(a, b, [] (bool r, T x, T y) { return r && x <= y; }, true, Indices{}); }

template <typename T, size_t N, typename... R>
inline VectorN<T, N> max(const VectorN<T, N> &a, const VectorN<T, N> &b, const VectorN<R, N> &... rest)
{ return max(max(a, b), rest...); }

template <typename T, size_t N, typename... R>
inline VectorN<T, N> min(const VectorN<T, N> &a, const VectorN<T, N> &b, const VectorN<R, N> &... rest)
{ return min(min(a, b), rest...); }

////////////////////////////////////////////////////////////////
/// AABB boolean impls
////////////////////////////////////////////////////////////////

template <typename T, size_t N>
inline bool is_valid(const Aabb<T, N> &b)
{ return b[0] <= b[1]; }

template <typename T, size_t N>
inline bool is_valid(const Aabb<T, N> &b, bool _)
{ return b[0] < b[1]; }

template <typename T, size_t N>
inline bool is_inside(const Aabb<T, N> &b, const VectorN<T, N> &v)
{ return b[0] <= v && v <= b[1]; }

template <typename T, size_t N>
inline bool is_inside(const Aabb<T, N> &b, const VectorN<T, N> &v, bool _)
{ return b[0] < v && v < b[1]; }

template <typename T, size_t N>
inline bool is_inside(const Aabb<T, N> &b_I, const Aabb<T, N> &b_i)
{ return b_I[0] <= b_i[0] && b_i[1] <= b_I[1]; }

template <typename T, size_t N>
inline bool is_inside(const Aabb<T, N> &b_I, const Aabb<T, N> &b_i, bool _)
{ return b_I[0] < b_i[0] && b_i[1] < b_I[1]; }

template <typename T, size_t N>
inline bool is_intersecting(const Aabb<T, N> &b_0, const Aabb<T, N> &b_1)
{ return b_0[0] <= b_1[1] && b_1[0] <= b_0[1]; }

template <typename T, size_t N>
inline bool is_intersecting(const Aabb<T, N> &b_0, const Aabb<T, N> &b_1, bool _)
{ return b_0[0] < b_1[1] && b_1[0] < b_0[1]; }

// * If we can rely on the IEEE 754 floating-point properties,
// this also implicitly handles the edge case where a component
// of the direction is zero - the tx0 and tx1 values (for example)
// will be infinities of opposite sign if the ray is within the slabs,
// thus leaving t0 and t1 unchanged.
// If the ray is outside the slabs, tx0 and tx1 will be infinities
// with the same sign, thus making t0 == +inf or t1 == -inf,
// and causing the test to fail.
// * As AABB is not the entity in space, its intersect test does not
// update distance.

template <typename T, size_t N>
inline bool is_intersecting(const Aabb<T, N> &b, const VectorN<T, N> &org, const VectorN<T, N> &dir, const T &dist)
{
    const VectorN<T, N> k0 = (b[0] - org) / dir;
    const VectorN<T, N> k1 = (b[1] - org) / dir;
    const T t0 = max(min(k0, k1));
    const T t1 = min(max(k0, k1));
    return t1 > 0 && t1 >= t0 && dist > t0;
}

template <typename T, size_t N>
inline bool is_intersecting(const Aabb<T, N> &b, const VectorN<T, N> &org, const VectorN<T, N> &inv, const T &dist, bool _)
{
    const VectorN<T, N> k0 = (b[0] - org) * inv;
    const VectorN<T, N> k1 = (b[1] - org) * inv;
    const T t0 = max(min(k0, k1));
    const T t1 = min(max(k0, k1));
    return t1 > 0 && t1 >= t0 && dist > t0;
}

////////////////////////////////////////////////////////////////
/// AABB property impls
////////////////////////////////////////////////////////////////

template <typename T, size_t N>
inline VectorN<T, N> centroid(const Aabb<T, N> &b)
{ return (b[0] + b[1]) * (T)(0.5); }

template <typename T, size_t N>
inline VectorN<T, N> diagonal(const Aabb<T, N> &b)
{ return b[1] - b[0]; }

template <typename T, size_t N>
inline T component(const Aabb<T, N> &b, size_t dim)
{ return diagonal(b)[dim]; }

template <typename T, size_t N>
inline T max_component(const Aabb<T, N> &b)
{ return max(diagonal(b)); }

template <typename T, size_t N>
inline size_t longest_axis(const Aabb<T, N> &b)
{ return argmax(diagonal(b)); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline T volume(const Aabb<T, N> &b)
{ return op_impl_rdc<T, N>(diagonal(b), [] (T x, T y) { return x * y; }, (T)1, Indices{}); }

////////////////////////////////////////////////////////////////
/// AABB operation impls
////////////////////////////////////////////////////////////////

template <typename T, size_t N>
inline Aabb<T, N> merge(const Aabb<T, N> &b_0, const Aabb<T, N> &b_1)
{ return { min(b_0[0], b_1[0]), max(b_0[1], b_1[1]) }; }

template <typename T, size_t N>
inline Aabb<T, N> intersect(const Aabb<T, N> &b_0, const Aabb<T, N> &b_1)
{ return { max(b_0[0], b_1[0]), min(b_0[1], b_1[1]) }; }

////////////////////////////////////////////////////////////////
/// AABB ctors
////////////////////////////////////////////////////////////////

#include <limits>

template <typename T, size_t N>
inline Aabb<T, N> make_aabb()
{ return { make_vector<T, N>(+std::numeric_limits<T>::max()), make_vector<T, N>(-std::numeric_limits<T>::max()) }; }

template <typename T, size_t N>
inline Aabb<T, N> make_aabb(const VectorN<T, N> &v)
{ return { v, v }; }

template <typename T, size_t N, typename... R>
inline Aabb<T, N> make_aabb(const VectorN<T, N> &v, const VectorN<R, N> &... vs)
{ return { min(v, vs...), max(v, vs...) }; }

////////////////////////////////////////////////////////////////
/// 3D AABB property impls
////////////////////////////////////////////////////////////////

template <typename T>
inline T area(const Aabb<T, 3> &b)
{
    VectorN<T, 3> d = diagonal(b);
    return (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]) * (T)2;
}

////////////////////////////////////////////////////////////////
/// 2D AABB property impls
////////////////////////////////////////////////////////////////

template <typename T>
inline T area(const Aabb<T, 2> &b)
{
    VectorN<T, 2> d = diagonal(b);
    return (d[0] + d[1]) * (T)2;
}

#endif // AXIS_ALIGNED_BOUNDING_BOX_IMPL_HH
