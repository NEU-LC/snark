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

#ifndef SNARK_IMAGING_CV_MAT_TYPE_TRAITS_H_
#define SNARK_IMAGING_CV_MAT_TYPE_TRAITS_H_

#include <opencv2/core/core.hpp>

namespace snark { namespace cv_mat {

template< typename T, int C, int S > struct base_type_traits
{
    typedef T value_t;
    static const int channels = C;
    static const int single_channel_type = S;
};

template< int type > struct type_traits;
template<> struct type_traits< CV_8UC1 > : base_type_traits< unsigned char, 1, CV_8UC1 > {};
template<> struct type_traits< CV_8UC2 > : base_type_traits< unsigned char, 2, CV_8UC1 > {};
template<> struct type_traits< CV_8UC3 > : base_type_traits< unsigned char, 3, CV_8UC1 > {};
template<> struct type_traits< CV_8UC4 > : base_type_traits< unsigned char, 4, CV_8UC1 > {};
template<> struct type_traits< CV_8SC1 > : base_type_traits< char, 1, CV_8SC1 > {};
template<> struct type_traits< CV_8SC2 > : base_type_traits< char, 2, CV_8SC1 > {};
template<> struct type_traits< CV_8SC3 > : base_type_traits< char, 3, CV_8SC1 > {};
template<> struct type_traits< CV_8SC4 > : base_type_traits< char, 4, CV_8SC1 > {};
template<> struct type_traits< CV_16UC1 > : base_type_traits< comma::uint16, 1, CV_16UC1 > {};
template<> struct type_traits< CV_16UC2 > : base_type_traits< comma::uint16, 2, CV_16UC1 > {};
template<> struct type_traits< CV_16UC3 > : base_type_traits< comma::uint16, 3, CV_16UC1 > {};
template<> struct type_traits< CV_16UC4 > : base_type_traits< comma::uint16, 4, CV_16UC1 > {};
template<> struct type_traits< CV_16SC1 > : base_type_traits< comma::int16, 1, CV_16SC1 > {};
template<> struct type_traits< CV_16SC2 > : base_type_traits< comma::int16, 2, CV_16SC1 > {};
template<> struct type_traits< CV_16SC3 > : base_type_traits< comma::int16, 3, CV_16SC1 > {};
template<> struct type_traits< CV_16SC4 > : base_type_traits< comma::int16, 4, CV_16SC1 > {};
template<> struct type_traits< CV_32SC1 > : base_type_traits< comma::int32, 1, CV_32SC1 > {};
template<> struct type_traits< CV_32SC2 > : base_type_traits< comma::int32, 2, CV_32SC1 > {};
template<> struct type_traits< CV_32SC3 > : base_type_traits< comma::int32, 3, CV_32SC1 > {};
template<> struct type_traits< CV_32SC4 > : base_type_traits< comma::int32, 4, CV_32SC1 > {};
template<> struct type_traits< CV_32FC1 > : base_type_traits< float, 1, CV_32FC1 > {};
template<> struct type_traits< CV_32FC2 > : base_type_traits< float, 2, CV_32FC1 > {};
template<> struct type_traits< CV_32FC3 > : base_type_traits< float, 3, CV_32FC1 > {};
template<> struct type_traits< CV_32FC4 > : base_type_traits< float, 4, CV_32FC1 > {};
template<> struct type_traits< CV_64FC1 > : base_type_traits< double, 1, CV_64FC1 > {};
template<> struct type_traits< CV_64FC2 > : base_type_traits< double, 2, CV_64FC1 > {};
template<> struct type_traits< CV_64FC3 > : base_type_traits< double, 3, CV_64FC1 > {};
template<> struct type_traits< CV_64FC4 > : base_type_traits< double, 4, CV_64FC1 > {};

} }  // namespace snark { namespace cv_mat {

#endif // SNARK_IMAGING_CV_MAT_TYPE_TRAITS_H_
