// ======================================================================== //
// Copyright (c) 2023 Ingram Inxent                                         //
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

#ifndef N_DIMENSIONAL_VECTOR_HH
#define N_DIMENSIONAL_VECTOR_HH

#if __cplusplus >= 201703L
#define ENABLED_CPP_STD_17
#endif

#ifndef _WIN64
#include <cstddef>
#endif

////////////////////////////////////////////////////////////////
/// Nd vector
////////////////////////////////////////////////////////////////

template <typename T, size_t N>
struct VectorN
{
    typedef T value_type;

    static constexpr size_t size() noexcept { return N; }

    inline const T &operator[](size_t i) const { return v[i]; }
    inline T &operator[](size_t i) { return v[i]; }

    inline operator const T *() const { return &v[0]; }
    inline operator T *() { return &v[0]; }

    T v[N];
};

////////////////////////////////////////////////////////////////
/// Nd vector internals
////////////////////////////////////////////////////////////////

#include <utility> // std::index_sequence

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_neg(const VectorN<T, N> &p, std::index_sequence<I...>)
{ return { (-p[I])... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_add(const VectorN<T, N> &a, const VectorN<T, N> &b, std::index_sequence<I...>)
{ return { (a[I] + b[I])... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_add(const VectorN<T, N> &p, const T &s, std::index_sequence<I...>)
{ return { (p[I] + s)... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_sub(const VectorN<T, N> &a, const VectorN<T, N> &b, std::index_sequence<I...>)
{ return { (a[I] - b[I])... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_sub(const VectorN<T, N> &p, const T &s, std::index_sequence<I...>)
{ return { (p[I] - s)... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_mul(const VectorN<T, N> &a, const VectorN<T, N> &b, std::index_sequence<I...>)
{ return { (a[I] * b[I])... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_mul(const VectorN<T, N> &p, const T &s, std::index_sequence<I...>)
{ return { (p[I] * s)... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_div(const VectorN<T, N> &a, const VectorN<T, N> &b, std::index_sequence<I...>)
{ return { (a[I] / b[I])... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_div(const VectorN<T, N> &p, const T &s, std::index_sequence<I...>)
{ return { (p[I] / s)... }; } // to optimize for large size

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_set(const T &s, std::index_sequence<I...>)
{ return { (s + (T)0 * I)... }; }

template <typename T, size_t N, size_t M, size_t... I>
inline VectorN<T, N> op_impl_set(const VectorN<T, M> &p, std::index_sequence<I...>)
{ return { (I < M ? p[I] : (T)0)... }; }

template <typename T, size_t N, size_t... I>
inline T op_impl_rdc(const VectorN<T, N> &p, std::index_sequence<I...>)
#ifdef ENABLED_CPP_STD_17
{ return ((p[I]) + ...); }
#else
{ using _ = int[]; T r {}; (void)_{ (r += p[I], 0)... }; return r; }
#endif

template <typename T, size_t N, size_t... I>
inline T op_impl_dot(const VectorN<T, N> &a, const VectorN<T, N> &b, std::index_sequence<I...>)
#ifdef ENABLED_CPP_STD_17
{ return ((a[I] * b[I]) + ...); }
#else
{ using _ = int[]; T r {}; (void)_{ (r += a[I] * b[I], 0)... }; return r; }
#endif

template <typename T, size_t N, class Stream, size_t... I>
inline Stream &op_impl_out(Stream &os, const VectorN<T, N>& p, const char *sep, std::index_sequence<I...>)
#ifdef ENABLED_CPP_STD_17
{ ((os << (I == 0 ? "" : sep) << p[I]), ...); return os; }
#else
{ using _ = int[]; (void)_{ (os << (I == 0 ? "" : sep) << p[I], 0)... }; return os; }
#endif

//template <typename T, size_t N, size_t... I>
//inline T op_impl_max(const VectorN<T, N> &p, std::index_sequence<I...>)
//{ using _ = int[]; T r = p[0]; (void)_{ (r = std::max(r, p[I]), 0)... }; return r; }

//template <typename T, size_t N, size_t... I>
//inline size_t op_impl_argmax(const VectorN<T, N> &p, std::index_sequence<I...>)
//{ using _ = int[]; size_t k {}; (void)_{ (k = p[k] < p[I] ? I : k, 0)... }; return k; }

//template <typename T, size_t N, size_t... I>
//inline T op_impl_min(const VectorN<T, N> &p, std::index_sequence<I...>)
//{ using _ = int[]; T r = p[0]; (void)_{ (r = std::min(r, p[I]), 0)... }; return r; }

//template <typename T, size_t N, size_t... I>
//inline size_t op_impl_argmin(const VectorN<T, N> &p, std::index_sequence<I...>)
//{ using _ = int[]; size_t k {}; (void)_{ (k = p[k] > p[I] ? I : k, 0)... }; return k; }

////////////////////////////////////////////////////////////////
/// Nd vector generic internals
////////////////////////////////////////////////////////////////

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_uop(const VectorN<T, N> &p, T (*fun)(T), std::index_sequence<I...>)
{ return { fun(p[I])... }; }

template <typename T, size_t N, typename R, size_t... I>
inline VectorN<R, N> op_impl_uop(const VectorN<T, N> &p, R (*fun)(T), std::index_sequence<I...>)
{ return { fun(p[I])... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_bop(const VectorN<T, N> &p, const T &s, T (*fun)(T, T), std::index_sequence<I...>)
{ return { fun(p[I], s)... }; }

template <typename T, size_t N, typename R, size_t... I>
inline VectorN<R, N> op_impl_bop(const VectorN<T, N> &p, const T &s, R (*fun)(T, T), std::index_sequence<I...>)
{ return { fun(p[I], s)... }; }

template <typename T, size_t N, size_t... I>
inline VectorN<T, N> op_impl_bop(const VectorN<T, N> &a, const VectorN<T, N> &b, T (*fun)(T, T), std::index_sequence<I...>)
{ return { fun(a[I], b[I])... }; }

template <typename T, size_t N, typename R, size_t... I>
inline VectorN<R, N> op_impl_bop(const VectorN<T, N> &a, const VectorN<T, N> &b, R (*fun)(T, T), std::index_sequence<I...>)
{ return { fun(a[I], b[I])... }; }

template <typename T, size_t N, size_t... I>
inline T op_impl_rdc(const VectorN<T, N> &p, T (*fun)(T), std::index_sequence<I...>)
{ using _ = int[]; T r {}; (void)_{ (r += fun(p[I]), 0)... }; return r; }

template <typename T, size_t N, size_t... I>
inline T op_impl_rdc(const VectorN<T, N> &p, T (*fun)(T, T), T init, std::index_sequence<I...>)
{ using _ = int[]; T r { init }; (void)_{ (r = fun(r, p[I]), 0)... }; return r; }

template <typename T, size_t N, typename R, size_t... I>
inline R op_impl_rdc(const VectorN<T, N> &p, R (*fun)(R, T), R init, std::index_sequence<I...>)
{ using _ = int[]; R r { init }; (void)_{ (r = fun(r, p[I]), 0)... }; return r; }

template <typename T, size_t N, size_t... I>
inline T op_impl_dot(const VectorN<T, N> &a, const VectorN<T, N> &b, T (*fun)(T, T), std::index_sequence<I...>)
{ using _ = int[]; T r {}; (void)_{ (r += fun(a[I], b[I]), 0)... }; return r; }

template <typename T, size_t N, size_t... I>
inline T op_impl_dot(const VectorN<T, N> &a, const VectorN<T, N> &b, T (*fun)(T, T, T), T init, std::index_sequence<I...>)
{ using _ = int[]; T r { init }; (void)_{ (r = fun(r, a[I], b[I]), 0)... }; return r; }

template <typename T, size_t N, typename R, size_t... I>
inline R op_impl_dot(const VectorN<T, N> &a, const VectorN<T, N> &b, R (*fun)(R, T, T), R init, std::index_sequence<I...>)
{ using _ = int[]; R r { init }; (void)_{ (r = fun(r, a[I], b[I]), 0)... }; return r; }

template <typename T, size_t N, size_t... I>
inline size_t op_impl_arg(const VectorN<T, N> &p, bool (*cmp)(T, T), std::index_sequence<I...>)
{ using _ = int[]; size_t k {}; (void)_{ (k = cmp(p[k], p[I]) ? I : k, 0)... }; return k; }

////////////////////////////////////////////////////////////////
/// Nd vector ctors
////////////////////////////////////////////////////////////////

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> make_vector(const T &s)
{ return op_impl_set<T, N>(s, Indices{}); }

template <typename T, size_t N, size_t M, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> make_vector(const VectorN<T, M> &p)
{ return op_impl_set<T, N, M>(p, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> make_vector(const VectorN<T, N> &p, T (*fun)(T))
{ return op_impl_uop<T, N>(p, fun, Indices{}); }

template <typename T, size_t N, typename R, typename Indices = std::make_index_sequence<N>>
inline VectorN<R, N> make_vector(const VectorN<T, N> &p, R (*fun)(T))
{ return op_impl_uop<T, N, R>(p, fun, Indices{}); }

////////////////////////////////////////////////////////////////
/// Nd vector basic operations
////////////////////////////////////////////////////////////////

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator+(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_add(a, b, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator+(const VectorN<T, N> &p, const T &s)
{ return op_impl_add(p, s, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator+(const T &s, const VectorN<T, N> &p)
{ return op_impl_add(p, s, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator-(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_sub(a, b, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator-(const VectorN<T, N> &p, const T &s)
{ return op_impl_sub(p, s, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator-(const T &s, const VectorN<T, N> &p)
{ return op_impl_neg(op_impl_sub(p, s, Indices{}), Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator*(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_mul(a, b, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator*(const VectorN<T, N> &p, const T &s)
{ return op_impl_mul(p, s, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator*(const T &s, const VectorN<T, N> &p)
{ return op_impl_mul(p, s, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator/(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_div(a, b, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator/(const VectorN<T, N> &p, const T &s)
{ return op_impl_div(p, s, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> operator-(const VectorN<T, N> &p)
{ return op_impl_neg(p, Indices{}); }

template <typename T, size_t N>
inline VectorN<T, N> &operator+=(VectorN<T, N> &a, const VectorN<T, N> &b)
{ a = a + b; return a; }

template <typename T, size_t N>
inline VectorN<T, N> &operator+=(VectorN<T, N> &p, const T &s)
{ p = p + s; return p; }

template <typename T, size_t N>
inline VectorN<T, N> &operator-=(VectorN<T, N> &a, const VectorN<T, N> &b)
{ a = a - b; return a; }

template <typename T, size_t N>
inline VectorN<T, N> &operator-=(VectorN<T, N> &p, const T &s)
{ p = p - s; return p; }

template <typename T, size_t N>
inline VectorN<T, N> &operator*=(VectorN<T, N> &p, const T &s)
{ p = p * s; return p; }

template <typename T, size_t N>
inline VectorN<T, N> &operator/=(VectorN<T, N> &p, const T &s)
{ p = p / s; return p; }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline T sum(const VectorN<T, N> &p)
{ return op_impl_rdc(p, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline T dot(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_dot(a, b, Indices{}); }

////////////////////////////////////////////////////////////////
/// Nd vector extended operations
////////////////////////////////////////////////////////////////

#include <cmath>
#include <iostream>
#include <algorithm>

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline T max(const VectorN<T, N> &p)
{ return op_impl_rdc<T, N>(p, [] (T x, T y) { return std::max(x, y); }, p[0], Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline size_t argmax(const VectorN<T, N> &p)
{ return op_impl_arg<T, N>(p, [] (T x, T y) { return x < y; }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline T min(const VectorN<T, N> &p)
{ return op_impl_rdc<T, N>(p, [] (T x, T y) { return std::min(x, y); }, p[0], Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline size_t argmin(const VectorN<T, N> &p)
{ return op_impl_arg<T, N>(p, [] (T x, T y) { return x > y; }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> max(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_bop<T, N>(a, b, [] (T x, T y) { return std::max(x, y); }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> min(const VectorN<T, N> &a, const VectorN<T, N> &b)
{ return op_impl_bop<T, N>(a, b, [] (T x, T y) { return std::min(x, y); }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> abs(const VectorN<T, N> &p)
{ return op_impl_uop<T, N>(p, [] (T x) { return std::abs(x); }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> pow(const VectorN<T, N> &p, const T &s)
{ return op_impl_bop<T, N>(p, s, [] (T x, T y) { return std::pow(x, y); }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> exp(const VectorN<T, N> &p)
{ return op_impl_uop<T, N>(p, [] (T x) { return std::exp(x); }, Indices{}); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline VectorN<T, N> log(const VectorN<T, N> &p)
{ return op_impl_uop<T, N>(p, [] (T x) { return std::log(x); }, Indices{}); }

template <typename T, size_t N>
inline T norm1(const VectorN<T, N> &p)
{ return sum(abs(p)); }

template <typename T, size_t N>
inline T norm2(const VectorN<T, N> &p)
{ return std::sqrt(dot(p, p)); }

template <typename T, size_t N>
inline T norm8(const VectorN<T, N> &p)
{ return max(abs(p)); }

template <typename T, size_t N>
inline T norm(const VectorN<T, N> &p, const T &s)
{ return std::pow(sum(pow(abs(p), s)), (T)1 / s); }

template <typename T, size_t N>
inline VectorN<T, N> normalize(const VectorN<T, N> &p)
{ return p / norm2(p); }

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
inline std::ostream &operator<<(std::ostream &os, const VectorN<T, N>& p)
{ os << "("; op_impl_out(os, p, ", ", Indices{}) << ")"; return os; }

////////////////////////////////////////////////////////////////
/// 3d vector Ops
////////////////////////////////////////////////////////////////

template <typename T>
inline VectorN<T, 3> cross(const VectorN<T, 3> &a, const VectorN<T, 3> &b)
{ return { a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0] }; }

////////////////////////////////////////////////////////////////
/// 2d vector Ops
////////////////////////////////////////////////////////////////

template <typename T>
inline T cross(const VectorN<T, 2> &a, const VectorN<T, 2> &b)
{ return a[0]*b[1] - a[1]*b[0]; }

#endif // N_DIMENSIONAL_VECTOR_HH
