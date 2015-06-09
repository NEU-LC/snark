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

/// @author vsevolod vlaskine

#ifndef SNARK_TERRAIN_DEM_SRTM_HEADER_H_
#define SNARK_TERRAIN_DEM_SRTM_HEADER_H_

#include <comma/base/types.h>
#include "../../../math/spherical_geometry/coordinates.h"

namespace snark { namespace terrain { namespace dem { namespace srtm {

struct header
{
    std::string byteorder;
    std::string layout;
    comma::uint32 nrows;
    comma::uint32 ncols;
    comma::uint32 nbands;
    comma::uint32 nbits;
    comma::uint32 bandrowbytes;
    comma::uint32 totalrowbytes;
    comma::uint32 bandgapbytes;
    double nodata;
    double ulxmap;
    double ulymap;
    double xdim;
    double ydim;
    
    header() : nrows( 0 ), ncols( 0 ), nbands( 0 ), nbits( 0 ), bandrowbytes( 0 ), totalrowbytes( 0 ), bandgapbytes( 0 ), nodata( 0 ), ulxmap( 0 ), ulymap( 0 ), xdim( 0 ), ydim( 0 ) {}
};

} } } } // namespace snark { namespace terrain { namespace dem { namespace srtm {

#endif // SNARK_TERRAIN_DEM_SRTM_HEADER_H_
