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
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <comma/base/exception.h>
#include "file.h"
#include "utils.h"

namespace snark { namespace cv_mat { namespace impl {
    
template < typename H >
file< H >::file( const get_timestamp_functor& get_timestamp
               , const std::string& type
               , bool no_header
               , const boost::optional< int >& quality
               , bool do_index
               , bool numbered
               , const std::vector< std::string >& filenames
               , const std::vector< std::pair< unsigned int, unsigned int > >& ranges )
    : get_timestamp_( get_timestamp )
    , type_( type )
    , quality_( quality )
    , do_index_( do_index )
    , numbered_( numbered )
    , index_( 0 )
    , filenames_( filenames )
    , filename_index_( 0 )
    , ranges_( ranges )
    , range_index_( 0 )
    , count_( 0 )
{
    snark::cv_mat::serialization::options options;
    options.no_header = no_header;
    serialization_ = snark::cv_mat::serialization( options );
}

template < typename H >
typename std::pair< H, cv::Mat > file< H >::operator()( typename std::pair< H, cv::Mat > m )
{
    if( m.second.empty() ) { return m; }
    if( !ranges_.empty() )
    {
        for( ; range_index_ < ranges_.size() && count_ >= ranges_[range_index_].second; ++range_index_ );
        if( range_index_ == ranges_.size() ) { return std::pair< H, cv::Mat >(); } // end of desired ranges; todo? simply continue?
        if( count_ < ranges_[range_index_].first ) { ++count_; return m; }
    }
    boost::posix_time::ptime timestamp = get_timestamp_( m.first );
    index_ = timestamp == previous_timestamp_ ? index_ + 1 : 0;
    previous_timestamp_ = timestamp;
    std::string filename = make_filename_( timestamp );
    if( filename.empty() ) { return std::pair< H, cv::Mat >(); } // end of list of filenames; todo? simply continue?
    if( type_ == "bin" )
    {
        std::ofstream ofs( filename );
        if( !ofs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
        serialization_.write( ofs, m );
    }
    else if( type_ == "gz" )
    {
        std::ofstream ofs( filename );
        if( !ofs.is_open() ) { COMMA_THROW( comma::exception, "failed to open '" << filename << "'" ); }
        std::ostringstream oss;
        serialization_.write( oss, m );
        std::istringstream iss( oss.str() ); // todo? watch performance; do zero-copy?
		boost::iostreams::filtering_streambuf< boost::iostreams::input > os;  // todo? watch performance; create output stream on construction
		os.push( boost::iostreams::gzip_compressor( boost::iostreams::gzip_params( boost::iostreams::gzip::best_compression ) ) );
		os.push( iss );
		boost::iostreams::copy( os, ofs );
    }
    else
    {
        check_image_type( m.second, type_ );
        if( !cv::imwrite( filename, m.second, quality_ ? imwrite_params( type_, *quality_ ) : std::vector< int >() ) ) { COMMA_THROW( comma::exception, "failed to write image to '" << filename << "'" ); }
    }
    ++count_;
    return m;
}

template < typename H >
std::string file< H >::make_filename_( const boost::posix_time::ptime& t )
{
    if( numbered_ ) { return boost::lexical_cast< std::string >( count_ ) + '.' + type_; }
    if( filenames_.empty() ) { return make_filename( t, type_, do_index_ ? boost::optional< unsigned int >( index_ ) : boost::none ); }
    return filename_index_ < filenames_.size() ? filenames_[filename_index_++] : "";
}

template class file< boost::posix_time::ptime >;
template class file< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
