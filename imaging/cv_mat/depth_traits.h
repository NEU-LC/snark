// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SNARK_IMAGING_CV_MAT_DEPTH_TRAITS_H_
#define SNARK_IMAGING_CV_MAT_DEPTH_TRAITS_H_

#include <opencv2/core/core.hpp>

namespace snark { namespace cv_mat {

template< typename T> struct base_depth_traits
{
    typedef T value_t;
};

template< int Depth > struct depth_traits;

template<> struct depth_traits< CV_8U  > : base_depth_traits< unsigned char > {
    static double max_value() { return UCHAR_MAX; }
    static double min_value() { return 0; }
    static const int depth = CV_8U;
};

template<> struct depth_traits< CV_8S  > : base_depth_traits< char> {
    static double max_value() { return  SCHAR_MAX; }
    static double min_value() { return  SCHAR_MIN; }
    static const int depth = CV_8S;
};

template<> struct depth_traits< CV_16U > : base_depth_traits< comma::uint16> {
    static double max_value() { return  USHRT_MAX; }
    static double min_value() { return  0; }
    static const int depth = CV_16U;
};

template<> struct depth_traits< CV_16S > : base_depth_traits< comma::int16> {
    static double max_value() { return  SHRT_MAX; }
    static double min_value() { return  SHRT_MIN; }
    static const int depth = CV_16S;
};

template<> struct depth_traits< CV_32S > : base_depth_traits< comma::int32> {
    static double max_value() { return  INT_MAX; }
    static double min_value() { return  INT_MIN; }
};

template<> struct depth_traits< CV_32F > : base_depth_traits< float> {
    static double max_value() { return  1; }
    static double min_value() { return  0; }
};

template<> struct depth_traits< CV_64F > : base_depth_traits< double> {
    static double max_value() { return  1; }
    static double min_value() { return  0; }
};

template< typename traits >
struct traits_to_depth;

template<> struct traits_to_depth< depth_traits< CV_8U >::value_t > { enum { value = CV_8U }; };
template<> struct traits_to_depth< depth_traits< CV_8S >::value_t > { enum { value = CV_8S }; };
template<> struct traits_to_depth< depth_traits< CV_16U >::value_t > { enum { value = CV_16U }; };
template<> struct traits_to_depth< depth_traits< CV_16S >::value_t > { enum { value = CV_16S }; };
template<> struct traits_to_depth< depth_traits< CV_32S >::value_t > { enum { value = CV_32S }; };
template<> struct traits_to_depth< depth_traits< CV_32F >::value_t > { enum { value = CV_32F }; };
template<> struct traits_to_depth< depth_traits< CV_64F >::value_t > { enum { value = CV_64F }; };

} } // namespace snark { namespace cv_mat {

#endif // SNARK_IMAGING_CV_MAT_DEPTH_TRAITS_H_
