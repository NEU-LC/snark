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

#include "load-impl.h"

#include <string>
#include <fstream>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include <comma/csv/format.h>
#include <opencv2/highgui/highgui.hpp>
#include "../serialization.h"

namespace snark{ namespace cv_mat { namespace impl {

template < typename H >
load_impl_< H >::load_impl_( const std::string& filename )
{
    const std::vector< std::string >& v = comma::split( filename, '.' );
    if( v.back() == "bin" ) // quick and dirty
    {
        std::ifstream ifs( &filename[0] );
        if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\"" ); }
        serialization s( "t,rows,cols,type", comma::csv::format( "t,3ui" ) ); // quick and dirty
        value = s.read< H >( ifs );
        ifs.close();
    }
    else
    {
        value.second = cv::imread( filename, -1 );
    }
    if( value.second.data == NULL ) { COMMA_THROW( comma::exception, "failed to load image from file \""<< filename << "\"" ); }
}

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::impl::load_impl_< boost::posix_time::ptime >;
template class snark::cv_mat::impl::load_impl_< std::vector< char > >;