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

#ifndef SNARK_SENSORS_JAI_PIXEL_TYPE_NAMES_H_
#define SNARK_SENSORS_JAI_PIXEL_TYPE_NAMES_H_

#include <boost/assign.hpp>
#include <boost/bimap.hpp>
#include <Jai_Factory.h>

namespace snark { namespace jai {

typedef boost::bimap< uint32_t, std::string > pixel_type_names_t;
static const pixel_type_names_t pixel_type_names = boost::assign::list_of< pixel_type_names_t::relation >
( J_GVSP_PIX_MONO8, "Mono8" )
( J_GVSP_PIX_MONO8_SIGNED, "Mono8Signed" )
( J_GVSP_PIX_MONO10, "Mono10" )
( J_GVSP_PIX_MONO10_PACKED, "Mono10Packed" )
( J_GVSP_PIX_MONO12, "Mono12" )
( J_GVSP_PIX_MONO12_PACKED, "Mono12Packed" )
( J_GVSP_PIX_MONO14, "Mono14" )
( J_GVSP_PIX_MONO16, "Mono16" )
( J_GVSP_PIX_BAYGR8, "BayerGR8" )
( J_GVSP_PIX_BAYRG8, "BayerRG8" )
( J_GVSP_PIX_BAYGB8, "BayerGB8" )
( J_GVSP_PIX_BAYBG8, "BayerBG8" )
( J_GVSP_PIX_BAYGR10, "BayerGR10" )
( J_GVSP_PIX_BAYRG10, "BayerRG10" )
( J_GVSP_PIX_BAYGB10, "BayerGB10" )
( J_GVSP_PIX_BAYBG10, "BayerBG10" )
( J_GVSP_PIX_BAYGR12, "BayerGR12" )
( J_GVSP_PIX_BAYRG12, "BayerRG12" )
( J_GVSP_PIX_BAYGB12, "BayerGB12" )
( J_GVSP_PIX_BAYBG12, "BayerBG12" )
( J_GVSP_PIX_BAYGR16, "BayerGR16" )
( J_GVSP_PIX_BAYRG16, "BayerRG16" )
( J_GVSP_PIX_BAYGB16, "BayerGB16" )
( J_GVSP_PIX_BAYBG16, "BayerBG16" )
( J_GVSP_PIX_BAYGR10_PACKED, "BayerGR10Packed" )
( J_GVSP_PIX_BAYRG10_PACKED, "BayerRG10Packed" )
( J_GVSP_PIX_BAYGB10_PACKED, "BayerGB10Packed" )
( J_GVSP_PIX_BAYBG10_PACKED, "BayerBG10Packed" )
( J_GVSP_PIX_BAYGR12_PACKED, "BayerGR12Packed" )
( J_GVSP_PIX_BAYRG12_PACKED, "BayerRG12Packed" )
( J_GVSP_PIX_BAYGB12_PACKED, "BayerGB12Packed" )
( J_GVSP_PIX_BAYBG12_PACKED, "BayerBG12Packed" )
( J_GVSP_PIX_RGB8_PACKED, "RGB8Packed" )
( J_GVSP_PIX_BGR8_PACKED, "BGR8Packed" )
( J_GVSP_PIX_RGBA8_PACKED, "RGBA8Packed" )
( J_GVSP_PIX_BGRA8_PACKED, "BGRA8Packed" )
( J_GVSP_PIX_RGB10_PACKED, "RGB10Packed" )
( J_GVSP_PIX_BGR10_PACKED, "BGR10Packed" )
( J_GVSP_PIX_RGB12_PACKED, "RGB12Packed" )
( J_GVSP_PIX_BGR12_PACKED, "BGR12Packed" )
( J_GVSP_PIX_RGB16_PACKED, "RGB16Packed" )
( J_GVSP_PIX_RGB10V1_PACKED, "RGB10V1Packed" )
( J_GVSP_PIX_RGB10V2_PACKED, "RGB10V2Packed" )
( J_GVSP_PIX_RGB12V1_PACKED, "RGB12V1Packed" )
( J_GVSP_PIX_YUV411_PACKED, "YUV411Packed" )
( J_GVSP_PIX_YUV422_PACKED, "YUV42Packed" )
( J_GVSP_PIX_YUV422_YUYV_PACKED, "YUYVPacked" )
( J_GVSP_PIX_YUV444_PACKED, "YUV444Packed" )
( J_GVSP_PIX_RGB8_PLANAR, "RGB8Planar" )
( J_GVSP_PIX_RGB10_PLANAR, "RGB10Planar" )
( J_GVSP_PIX_RGB12_PLANAR, "RGB12Planar" )
( J_GVSP_PIX_RGB16_PLANAR, "RGB16Planar" )
( J_GVSP_PIX_BGR16_PACKED_INTERNAL, "JAI SDK Internal 16-bit RGB pixel format" );

} } // namespace snark { namespace jai {

#endif // SNARK_SENSORS_JAI_PIXEL_TYPE_NAMES_H_