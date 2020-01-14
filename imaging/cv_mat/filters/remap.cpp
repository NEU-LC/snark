// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

#include <fstream>
#include <boost/date_time/posix_time/ptime.hpp>
#include <comma/base/exception.h>
#include <comma/string/string.h>
#include <comma/name_value/serialize.h>
#include "../../camera/pinhole.h"
#include "../../camera/traits.h"
#include "remap.h"

namespace snark { namespace cv_mat { namespace impl {

template < typename H >
remap< H >::remap( const std::string& filename, unsigned int width, unsigned int height, int interpolation )
    : interpolation_( interpolation )
    , x_( int( height ), int( width ), CV_32FC1 )
    , y_( int( height ), int( width ), CV_32FC1 )
{
    std::ifstream ifs( filename );
    if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
    unsigned int size = width * height * 4;
    ifs.read( reinterpret_cast< char* >( x_.ptr() ), size );
    if( ifs.gcount() != int( size ) ) { COMMA_THROW( comma::exception, "on reading x map: expected " << size << " bytes, got: " << ifs.gcount() ); }
    ifs.read( reinterpret_cast< char* >( y_.ptr() ), size );
    if( ifs.gcount() != int( size ) ) { COMMA_THROW( comma::exception, "on reading y map: expected " << size << " bytes, got: " << ifs.gcount() ); }
    ifs.peek(); // quick and dirty
    if( !ifs.eof() ) { COMMA_THROW( comma::exception, "expected " << ( size * 2 ) << " bytes in \"" << filename << "\", got more bytes; invalid map size" ); }
}

template < typename H >
typename remap< H >::value_type remap< H >::operator()( value_type m ) const
{ 
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    cv::remap( m.second, n.second, x_, y_, interpolation_ );
    return n;
}

template < typename H >
undistort< H >::undistort( const std::string& filename, bool do_remap, int interpolation )
    : distortion_coefficients_( 0, 0, 0, 0, 0 )
    , interpolation_( interpolation )
{
    if( do_remap ) { filename_ = filename; return; }
    try
    {
        const std::vector< std::string >& v = comma::split( filename, ':' );
        snark::camera::pinhole::config_t config;
        switch( v.size() )
        {
            case 1: comma::read( config, filename ); break;
            case 2: comma::read( config, v[0], v[1] ); break;
            default: COMMA_THROW( comma::exception, "undistort: expected <filename>[:<path>]; got: \"" << filename << "\"" );
        }
        if( config.distortion )
        {
            if( config.distortion->map_filename.empty() )
            {
                distortion_coefficients_ = config.distortion->as< cv::Vec< double, 5 > >();
                camera_matrix_ = config.camera_matrix();
            }
            else
            {
                filename_ = config.distortion->map_filename;
            }
        }
    }
    catch( ... )
    {
        filename_ = filename;
    }
}

template < typename H >
typename undistort< H >::value_type undistort< H >::operator()( typename undistort< H >::value_type m )
{
    if( filename_.empty() )
    {
        value_type n;
        n.first = m.first;
        if( camera_matrix_.empty() ) { n.second = m.second.clone(); }
        else { cv::undistort( m.second, n.second, camera_matrix_, distortion_coefficients_ ); }
        return n;
    }
    if( !remap_ ) { remap_.reset( remap< H >( filename_, m.second.cols, m.second.rows, interpolation_ ) ); }
    return remap_->operator()( m );
}

template class snark::cv_mat::impl::remap< boost::posix_time::ptime >;
template class snark::cv_mat::impl::remap< std::vector< char > >;
template class snark::cv_mat::impl::undistort< boost::posix_time::ptime >;
template class snark::cv_mat::impl::undistort< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
