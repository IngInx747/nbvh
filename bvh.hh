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

#ifndef BOUNDING_VOLUME_HIERARCHY_HH
#define BOUNDING_VOLUME_HIERARCHY_HH

#include <stack>
#include <vector>
#include <algorithm>
#include "aabb.hh"

////////////////////////////////////////////////////////////////
/// Bvh node
////////////////////////////////////////////////////////////////

/// When representing inner node, i0, i1 = index of left and right
/// child nodes in the node array respectively;
/// When representing leaf node, i0, i1 = beginning index of object
/// in the primitive array and NEGATIVE number of objects.
template <typename T, size_t N>
struct BvhNode
{
    Aabb<T, N> b;
    int i0 { 0 };
    int i1 { 0 };
};

////////////////////////////////////////////////////////////////
/// Bounding volume hierarchy
////////////////////////////////////////////////////////////////

template <class Primitive, typename T, size_t N>
class Bvh
{
public:
    typedef T value_type;

public:
    template <class PrimitiveBound, class PrimitiveSplit>
    inline void build( // primitives are moved
        std::vector<Primitive> &primitives,
        const PrimitiveBound &bound,
        const PrimitiveSplit &split,
        const int threshold = 1);

    template <class PrimitiveBound, class PrimitiveSplit>
    inline void build( // primitives are copied
        typename std::vector<Primitive>::iterator biter,
        typename std::vector<Primitive>::iterator eiter,
        const PrimitiveBound &bound,
        const PrimitiveSplit &split,
        const int threshold = 1);

protected:
    template <class PrimitiveBound, class PrimitiveSplit>
    inline void recursive_build(
        typename std::vector<Primitive>::iterator biter,
        typename std::vector<Primitive>::iterator eiter,
        const int current_node_id,
        const int current_tree_depth,
        const PrimitiveBound &bound,
        const PrimitiveSplit &split,
        const int threshold);

public:
    template <class PrimitiveCollide>
    inline bool intersect(
        PrimitiveCollide &collide,
        const VectorN<T, N> &org,
        const VectorN<T, N> &dir,
        T &dist) const;

    template <class RangeQuery>
    inline bool search(RangeQuery &range) const;

    inline std::vector<Primitive> &primitives() { return mPrimitives; }
    inline const std::vector<Primitive> &primitives() const { return mPrimitives; }

    inline std::vector<BvhNode<T, N>> &nodes() { return mNodes; }
    inline const std::vector<BvhNode<T, N>> &nodes() const { return mNodes; }

    inline Aabb<T, N> aabb() const { return mNodes.size() > 0 ? mNodes[0].b : make_aabb<T, N>(); }
    inline bool is_empty() const { return mNodes.empty(); }

protected:
    std::vector<Primitive> mPrimitives;
    std::vector<BvhNode<T, N>> mNodes;
};

/// Bound Program Interfaces:
/// 
/// struct PrimitiveBound
/// {
///     Aabb operator() (const Primitive &primitive);
///     ...
/// };
/// 

/// Split Program Interfaces:
/// 
/// struct PrimitiveSplit
/// {
///     using Iter = typename std::vector<Primitive>::iterator;
///     Iter operator() (std::vector<Primitive> &primitives, Iter biter, Iter eiter) const;
///     ...
/// };
/// 
/// Some built-in implementations are provided.
/// 

/// Query Program Interfaces:
/// 
/// struct RangeQuery
/// {
///     bool operator() (const Aabb &Aabb); // rough query
///     bool operator() (const Primitive &primitive); // fine query
///     ...
/// };
/// 

/// Collide Program Interfaces:
/// 
/// struct PrimitiveCollide
/// {
///     bool operator() (const Primitive &primitive,
///                      const VectorN<T, N> &org,
///                      const VectorN<T, N> &dir,
///                      T &dist);
///     ...
/// };
/// 

////////////////////////////////////////////////////////////////
/// Bvh Build example
////////////////////////////////////////////////////////////////

/// template<class Primitive, class PrimitiveBound, typename T, size_t N>
/// void build_bvh_with_SAH_method(
///     Bvh<Primitive, T, N> &bvh,
///     const std::vector<Primitive> &primitives,
///     const PrimitiveBound &bound,
///     int threshold)
/// {
///     SAHSplit<Primitive, PrimitiveBound, T> split(bound);
///     bvh.build<PrimitiveBound, decltype(split)>(
///         primitives.begin(), primitives.end(),
///         bound, split, threshold);
/// }

/// template<class Primitive, class PrimitiveBound, typename T, size_t N>
/// void build_bvh_with_EC_method(
///     Bvh<Primitive, T, N> &bvh,
///     const std::vector<Primitive> &primitives,
///     const PrimitiveBound &bound,
///     int threshold)
/// {
///     EqualCountsSplit<Primitive, PrimitiveBound, T, N> split(bound);
///     bvh.build<PrimitiveBound, decltype(split)>(
///         primitives.begin(), primitives.end(),
///         bound, split, threshold);
/// }

#endif // !BOUNDING_VOLUME_HIERARCHY_HH

#include "bvh.impl.hh"
