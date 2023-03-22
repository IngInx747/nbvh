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

#ifndef BOUNDING_VOLUME_HIERARCHY_IMPL_HH
#define BOUNDING_VOLUME_HIERARCHY_IMPL_HH

#include "bvh.hh"

////////////////////////////////////////////////////////////////
/// Bvh node
////////////////////////////////////////////////////////////////

template <typename T, size_t N>
inline int &left_child(BvhNode<T, N> &node)
{ return node.i0; }

template <typename T, size_t N>
inline const int &left_child(const BvhNode<T, N> &node)
{ return node.i0; }

template <typename T, size_t N>
inline int &right_child(BvhNode<T, N> &node)
{ return node.i1; }

template <typename T, size_t N>
inline const int &right_child(const BvhNode<T, N> &node)
{ return node.i1; }

template <typename T, size_t N>
inline int &offset(BvhNode<T, N> &node)
{ return node.i0; }

template <typename T, size_t N>
inline const int &offset(const BvhNode<T, N> &node)
{ return node.i0; }

template <typename T, size_t N>
inline int &neglen(BvhNode<T, N> &node)
{ return node.i1; }

template <typename T, size_t N>
inline const int &neglen(const BvhNode<T, N> &node)
{ return node.i1; }

template <typename T, size_t N>
inline int length(const BvhNode<T, N> &node)
{ return -node.i1; }

template <typename T, size_t N>
inline bool is_leaf(const BvhNode<T, N> &node)
{ return neglen(node) < 0; }

template <typename T, size_t N>
inline void set_leaf(BvhNode<T, N> &node, int objIdx, int objNum)
{ offset(node) = objIdx; neglen(node) = -objNum; }

////////////////////////////////////////////////////////////////
/// Bvh build
////////////////////////////////////////////////////////////////

template <class Primitive, typename T, size_t N>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive, T, N>::recursive_build(
    typename std::vector<Primitive>::iterator biter,
    typename std::vector<Primitive>::iterator eiter,
    const int curr,  // current bvh node id
    const int depth, // current tree depth
    const PrimitiveBound &bound,
    const PrimitiveSplit &split,
    const int threshold)
{
    const int n = static_cast<int>(std::distance(biter, eiter));
    const int m = static_cast<int>(std::distance(mPrimitives.begin(), biter));

    // Split primitives into left and right children nodes at splitting index
    auto piter = eiter;
    if (n > threshold) piter = split(mPrimitives, biter, eiter);

    // Make Bvh leaf node if:
    // 1. #primitive is less than threshold, there is no need to split anymore;
    // 2. split method failed to split primitives into 2 sets(which causes #primitive > threshold).
    // To make #primitive per node strictly less than threshold, one needs a split method that
    // will certainly perform a successful split, like EqualCount method.
    if (piter == biter || piter == eiter)
    {
        set_leaf(mNodes[curr], m, n);
        auto bbox = make_aabb<T, N>();
        for (auto iter = biter; iter != eiter; ++iter)
            bbox = merge(bbox, bound(*iter));
        mNodes[curr].b = bbox;
    }
    else // Build Bvh recursively after splitting primitives
    {
        mNodes[curr].b = make_aabb<T, N>();

        int left = static_cast<int>(mNodes.size());
        left_child(mNodes[curr]) = left;
        mNodes.emplace_back();

        recursive_build(biter, piter, left, depth + 1, bound, split, threshold);
        mNodes[curr].b = merge(mNodes[curr].b, mNodes[left].b);

        int right = static_cast<int>(mNodes.size());
        right_child(mNodes[curr]) = right;
        mNodes.emplace_back();

        recursive_build(piter, eiter, right, depth + 1, bound, split, threshold);
        mNodes[curr].b = merge(mNodes[curr].b, mNodes[right].b);
    }
}

template <class Primitive, typename T, size_t N>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive, T, N>::build(
    std::vector<Primitive> &primitives,
    const PrimitiveBound &bound,
    const PrimitiveSplit &split,
    const int threshold)
{
    if (primitives.empty()) return;
    std::swap(primitives, mPrimitives); mNodes.emplace_back();
    recursive_build(mPrimitives.begin(), mPrimitives.end(), 0, 0, bound, split, threshold);
}

template <class Primitive, typename T, size_t N>
template <class PrimitiveBound, class PrimitiveSplit>
inline void Bvh<Primitive, T, N>::build(
    typename std::vector<Primitive>::iterator biter,
    typename std::vector<Primitive>::iterator eiter,
    const PrimitiveBound &bound,
    const PrimitiveSplit &split,
    const int threshold)
{
    if (primitives.empty()) return;
    mPrimitives.clear(); mNodes.emplace_back();
    std::copy(biter, eiter, std::back_inserter(mPrimitives));
    recursive_build(mPrimitives.begin(), mPrimitives.end(), 0, 0, bound, split, threshold);
}

////////////////////////////////////////////////////////////////
/// Bvh query
////////////////////////////////////////////////////////////////

template <class Primitive, typename T, size_t N>
template <class RangeQuery>
inline bool Bvh<Primitive, T, N>::search(RangeQuery &query) const
{
    if (mNodes.empty()) return false;

    bool hit { false };
    std::stack<int> recursive({ 0 });

    while (!recursive.empty())
    {
        int curr = recursive.top(); recursive.pop();
        const auto &node = mNodes[curr]; // safe reference

        if (query(node.b))
        {
            if (is_leaf(node))
            {
                int ib = offset(node);
                int ie = ib + length(node);
                for (int i = ib; i < ie; ++i)
                    if (query(mPrimitives[i]))
                        hit = true;
            }
            else
            {
                recursive.push(right_child(node));
                recursive.push(left_child(node));
            }
        }
    }

    return hit;
}

template <class Primitive, typename T, size_t N>
template <class PrimitiveCollide>
inline bool Bvh<Primitive, T, N>::intersect(
    PrimitiveCollide &collide,
    const VectorN<T, N> &org,
    const VectorN<T, N> &dir,
    T &dist) const
{
    if (mNodes.empty()) return false;

    const auto neg = make_vector<T, N, bool>(dir, [] (T x) { return x < 0; });
    const auto inv = make_vector<T, N>(1) / dir;

    bool hit { false };
    std::stack<int> recursive({ 0 });

    while (!recursive.empty())
    {
        int curr = recursive.top(); recursive.pop();
        const auto &node = mNodes[curr]; // safe reference

        if (is_intersecting(node.b, org, inv, dist, true))
        {
            if (is_leaf(node))
            {
                int ib = offset(node);
                int ie = ib + length(node);
                for (int i = ib; i < ie; ++i)
                    if (collide(mPrimitives[i], org, dir, dist))
                        hit = true;
            }
            else
            {
                const auto dim = longest_axis(node.b);

                if (neg[dim])
                {
                    recursive.push(left_child(node));
                    recursive.push(right_child(node));
                }
                else
                {
                    recursive.push(right_child(node));
                    recursive.push(left_child(node));
                }
            }
        }
    }

    return hit;
}

////////////////////////////////////////////////////////////////
/// Bvh Split Methods
////////////////////////////////////////////////////////////////

/// Split Method: EqualCounts
/// Partition primitives into equally-sized subsets
template<class Primitive, class PrimitiveBound, typename T, size_t N>
struct EqualCountsSplit
{
    EqualCountsSplit(const PrimitiveBound &bound): bound(bound) {}
    inline typename std::vector<Primitive>::iterator operator() (
        std::vector<Primitive> &primitives,
        typename std::vector<Primitive>::iterator biter,
        typename std::vector<Primitive>::iterator eiter) const;
    const PrimitiveBound &bound;
};

template<class Primitive, class PrimitiveBound, typename T, size_t N>
inline typename std::vector<Primitive>::iterator
EqualCountsSplit<Primitive, PrimitiveBound, T, N>::operator()(
    std::vector<Primitive> &primitives,
    typename std::vector<Primitive>::iterator biter,
    typename std::vector<Primitive>::iterator eiter) const
{
    auto cbox = make_aabb<T, N>(); // centroid bounding box
    for (auto iter = biter; iter != eiter; ++iter)
        cbox = merge(cbox, bound(*iter));
    const auto dim = longest_axis(cbox);

    const auto n = std::distance(biter, eiter);
    auto piter = biter + n / 2;

    std::nth_element(biter, piter, eiter, [&](const Primitive &a, const Primitive &b)
    { return centroid(bound(a))[dim] < centroid(bound(b))[dim]; });

    return piter;
}

/// Split Method: MiddlePoint
/// Partition primitives through node's midpoint
template<class Primitive, class PrimitiveBound, typename T, size_t N>
struct MiddlePointSplit
{
    MiddlePointSplit(const PrimitiveBound &bound) : bound(bound) {}
    inline typename std::vector<Primitive>::iterator operator() (
        std::vector<Primitive> &primitives,
        typename std::vector<Primitive>::iterator biter,
        typename std::vector<Primitive>::iterator eiter) const;
    const PrimitiveBound &bound;
};

template<class Primitive, class PrimitiveBound, typename T, size_t N>
inline typename std::vector<Primitive>::iterator
MiddlePointSplit<Primitive, PrimitiveBound, T, N>::operator()(
    std::vector<Primitive> &primitives,
    typename std::vector<Primitive>::iterator biter,
    typename std::vector<Primitive>::iterator eiter) const
{
    auto cbox = make_aabb<T, N>(); // centroid bounding box
    for (auto iter = biter; iter != eiter; ++iter)
        cbox = merge(cbox, bound(*iter));
    const auto dim = longest_axis(cbox);

    T mid = (cbox.p[0][dim] + cbox.p[1][dim]) * (T)(0.5);

    auto miter = std::partition(biter, eiter, [&](const Primitive &p)
    { return centroid(bound(p))[dim] < mid; });

    // use EqualCount if split failed
    if (miter == biter || miter == eiter)
    {
        const auto n = std::distance(biter, eiter);
        auto piter = biter + n / 2;
        std::nth_element(biter, piter, eiter, [&](const Primitive &a, const Primitive &b)
        { return centroid(bound(a))[dim] < centroid(bound(b))[dim]; });
        return piter;
    }

    return miter;
}

/// Split Method: SAH
/// Partition primitives via surface area heuristic
template<class Primitive, class PrimitiveBound, typename T, size_t N>
struct SAHSplit
{
    SAHSplit(const PrimitiveBound &bound) : bound(bound) {}
    inline typename std::vector<Primitive>::iterator operator() (
        std::vector<Primitive> &primitives,
        typename std::vector<Primitive>::iterator biter,
        typename std::vector<Primitive>::iterator eiter) const;
    const PrimitiveBound &bound;
    int nBuckets { 16 };
};

template<class Primitive, class PrimitiveBound, typename T, size_t N>
inline typename std::vector<Primitive>::iterator
SAHSplit<Primitive, PrimitiveBound, T, N>::operator()(
    std::vector<Primitive> &primitives,
    typename std::vector<Primitive>::iterator biter,
    typename std::vector<Primitive>::iterator eiter) const
{
    auto cbox = make_aabb<T, N>(); // centroid bounding box
    for (auto iter = biter; iter != eiter; ++iter)
        cbox = merge(cbox, bound(*iter));
    const auto dim = longest_axis(cbox);

    std::vector<Aabb<T, N>> boxes(nBuckets, make_aabb<T, N>());
    std::vector<int> counts(nBuckets, 0);

    for (auto iter = biter; iter != eiter; ++iter)
    {
        auto offset = (centroid(bound(*iter)) - cbox.p[0]) / diagonal(cbox);
        int b = static_cast<int>(nBuckets * offset[dim]);
        if (b == nBuckets) b = nBuckets - 1;
        boxes[b] = merge(boxes[b], bound(*iter));
        ++counts[b];
    }

    // cost of splitting [0,b] and [b+1, nB-1]
    T minCost = std::numeric_limits<T>::max();
    int splitBucketId = 0;

    for (int b = 0; b < nBuckets - 1; ++b)
    {
        Aabb<T, N> bbox0 = make_aabb<T, N>(), bbox1 = make_aabb<T, N>();
        int count0{}, count1{};

        for (int i = 0; i <= b; ++i)
        {
            bbox0 = merge(bbox0, boxes[i]);
            count0 += counts[i];
        }

        for (int i = b + 1; i < nBuckets; ++i)
        {
            bbox1 = merge(bbox1, boxes[i]);
            count1 += counts[i];
        }

        T cost = area(bbox0) * count0 + area(bbox1) * count1;

        // find bucket id that minimizes SAH metric
        if (minCost > cost)
        {
            minCost = cost;
            splitBucketId = b;
        }
    }

    // split according to the SAH result
    auto siter = std::partition(biter, eiter, [&](const Primitive &p)
    {
        auto offset = (centroid(bound(p)) - cbox.p[0]) / diagonal(cbox);
        int b = static_cast<int>(nBuckets * offset[dim]);
        if (b == nBuckets) b = nBuckets - 1;
        return b <= splitBucketId;
    });

    // use EqualCount if split failed
    if (siter == biter || siter == eiter)
    {
        const auto n = std::distance(biter, eiter);
        auto piter = biter + n / 2;
        std::nth_element(biter, piter, eiter, [&](const Primitive &a, const Primitive &b)
        { return centroid(bound(a))[dim] < centroid(bound(b))[dim]; });
        return piter;
    }

    return siter;
}

#endif // BOUNDING_VOLUME_HIERARCHY_IMPL_HH
