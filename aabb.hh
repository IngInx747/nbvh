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

#ifndef AXIS_ALIGNED_BOUNDING_BOX_HH
#define AXIS_ALIGNED_BOUNDING_BOX_HH

#include "nvec/nvec.hh"

////////////////////////////////////////////////////////////////
/// Axis-aligned Bounding Box
////////////////////////////////////////////////////////////////

template <typename T, size_t N>
struct Aabb
{
    typedef T value_type;
    typedef VectorN<T, N> type;

    const VectorN<T, N> &operator[](size_t i) const { return p[i]; }
    VectorN<T, N> &operator[](size_t i) { return p[i]; }

    VectorN<T, N> p[2]; // p_min, p_max
};

#endif // !AXIS_ALIGNED_BOUNDING_BOX_HH

#include "aabb.impl.hh"
