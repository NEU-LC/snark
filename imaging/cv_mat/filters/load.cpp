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

#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/string/split.h>
#include "../serialization.h"
#include "load.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
load< H >::load( const std::string& filename )
{
    auto extension = comma::split( filename, '.' ).back();
    try
    {
        if( extension.empty() || extension == "bin" ) // quick and dirty
        {
            std::ifstream ifs( &filename[0] );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open \"" << filename << "\"" ); }
            serialization s( "t,rows,cols,type", comma::csv::format( "t,3ui" ) ); // quick and dirty
            _value = s.read< H >( ifs );
            ifs.close();
        }
        else
        {
            _value.second = cv::imread( filename, -1 );
            if( _value.second.data == NULL ) { _video_capture.open( filename ); }
        }
    }
    catch( std::exception& ex ) { COMMA_THROW( comma::exception, "failed to load image or video from file \""<< filename << "\" (" << ex.what() << ")" ); }
    catch( ... ) { COMMA_THROW( comma::exception, "failed to load image or video from file \""<< filename << "\" (unknown exception)" ); }
}

template < typename H > bool load< H >::is_stream() const { return _value.second.empty(); }

template < typename H >
typename load< H >::value_type load< H >::operator()( typename load< H >::value_type m )
{
    if( !_value.second.empty() ) { return std::make_pair( _value.first, _value.second.clone() ); }
    load< H >::value_type n;
    n.first = m.first;
    _video_capture >> n.second;
    return n;
}

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::filters::load< boost::posix_time::ptime >;
template class snark::cv_mat::filters::load< std::vector< char > >;