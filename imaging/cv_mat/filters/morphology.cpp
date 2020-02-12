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

#include <iostream>
#include <memory>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <tbb/parallel_for.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/exception.h>
#include <comma/string/string.h>
#include "../utils.h"
#include "morphology.h"

namespace snark{ namespace cv_mat { namespace morphology {
    

const std::map< std::string, int >& operations()
{

    static const std::map< std::string, int > operations = { { "erode", cv::MORPH_ERODE }
                                                           , { "erosion", cv::MORPH_ERODE }
                                                           , { "dilate", cv::MORPH_DILATE }
                                                           , { "dilation", cv::MORPH_DILATE }
                                                           , { "open", cv::MORPH_OPEN }
                                                           , { "opening", cv::MORPH_OPEN }
                                                           , { "close", cv::MORPH_CLOSE }
                                                           , { "closing", cv::MORPH_CLOSE }
                                                           , { "gradient", cv::MORPH_GRADIENT }
                                                           , { "tophat", cv::MORPH_TOPHAT }
                                                           , { "blackhat", cv::MORPH_BLACKHAT } };
    return operations;
}

parameters::parameters( const std::vector< std::string >& e )
    : iterations( 1 )
{
    if( e.size() < 2 ) { return; }
    std::vector< std::string > p = comma::split( e[1], ',' );
    std::string eltype = p[0];
    try
    {
        if( eltype == "rectangle" || eltype == "ellipse" || eltype == "cross" ) 
        {
            if ( p.size() != 5 && p.size() != 3 && p.size() != 6 ) { COMMA_THROW( comma::exception, "structuring element of " << eltype << " type for the " << e[0] << " operation takes either 2, or 4 or 5 parameters" ); }
            while ( p.size() < 6 ) { p.push_back( "" ); }
            int size_x = ( p[1].empty() ? 3 : boost::lexical_cast< int >( p[1] ) );
            int size_y = ( p[2].empty() ? size_x : boost::lexical_cast< int >( p[2] ) );
            if ( size_x == 1 && size_y == 1 ) { std::cerr << "parse_structuring_element: warning: structuring element of a single point, no transformation is applied" << std::endl; }
            int anchor_x = ( p[3].empty() ? -1 : boost::lexical_cast< int >( p[3] ) );
            int anchor_y = ( p[4].empty() ? anchor_x : boost::lexical_cast< int >( p[4] ) );
            int shape = ( eltype == "rectangle" ? cv::MORPH_RECT : ( eltype == "ellipse" ? cv::MORPH_ELLIPSE : cv::MORPH_CROSS ) );
            kernel = cv::getStructuringElement( shape, cv::Size( size_x, size_y ), cv::Point( anchor_x, anchor_y ) );
            iterations = p[5].empty() ? 1 : boost::lexical_cast< comma::uint32 >( p[5] );
        } 
        else if( eltype == "square" || eltype == "circle" ) 
        {
            if ( p.size() > 4 ) { COMMA_THROW( comma::exception, "structuring element of " << eltype << " type for the " << e[0] << " operation takes either 0, or 1, or 2, or 3 parameters" ); }
            while ( p.size() < 4 ) { p.push_back( "" ); }
            int size_x = ( p[1].empty() ? 3 : boost::lexical_cast< int >( p[1] ) );
            if ( size_x == 1 ) { std::cerr << "parse_structuring_element: warning: structuring element of a single point, no transformation is applied" << std::endl; }
            int anchor_x = ( p[2].empty() ? -1 : boost::lexical_cast< int >( p[2] ) );
            int shape = ( eltype == "square" ? cv::MORPH_RECT : cv::MORPH_ELLIPSE );
            kernel = cv::getStructuringElement( shape, cv::Size( size_x, size_x ), cv::Point( anchor_x, anchor_x ) );
            iterations = p[3].empty() ? 1 : boost::lexical_cast< comma::uint32 >( p[3] );
        }
        else
        {
            COMMA_THROW( comma::exception, "the '" << eltype << "' type of the structuring element is not one of rectangle,square,ellipse,circle,cross" );
        }
    }
    catch( boost::bad_lexical_cast blc )
    {
        COMMA_THROW( comma::exception, "failed to cast parameter(s) for structuring element '" << eltype << "': " << blc.what() );
    }
}

template < typename H >
skeleton< H >::skeleton( const parameters& p ): kernel_( p.kernel ), iterations_( p.iterations ) {}

template < typename H >
typename skeleton< H >::value_type skeleton< H >::operator()( value_type m ) 
{
    if ( m.second.channels() != 1 ) { COMMA_THROW( comma::exception, "skeleton operations supports only single-channel (grey-scale) images" ); }
    typename std::pair< H, cv::Mat > result( m.first, cv::Mat( m.second.size(), CV_8UC1, cv::Scalar(0) ) );
    cv::Mat temp, eroded, img;
    m.second.copyTo( img );
    bool done = false;
    size_t iter = 0;
    do
    {
        cv::erode( img, eroded, kernel_, cv::Point(-1,-1), iterations_ );
        cv::dilate( eroded, temp, kernel_, cv::Point(-1,-1), iterations_ );
        cv::subtract( img, temp, temp );
        cv::bitwise_or( result.second, temp, result.second );
        eroded.copyTo( img );
        double min, max;
        cv::minMaxLoc( img, &min, &max );
        done = ( min == max );
        if ( ++iter > 1000 ) { COMMA_THROW( comma::exception, "skeleton did not converge after " << iter << " iterations" ); }
    } while ( !done );
    return result;
}

static bool advance_by_nearest( unsigned char* dest, const unsigned char* src, const unsigned char* neighbour, const unsigned char* background, unsigned int size )
{
    if( std::memcmp( src, background, size ) != 0 || std::memcmp( neighbour, background, size ) == 0 ) { return false; }
    std::memcpy( dest, neighbour, size );
    return true;
}

static bool retreat_by_nearest( unsigned char* dest, const unsigned char* src, const unsigned char* neighbour, const unsigned char* background, unsigned int size )
{
    if( std::memcmp( src, background, size ) == 0 || std::memcmp( neighbour, background, size ) != 0 ) { return false; }
    std::memcpy( dest, background, size );
    return true;
}

// static bool advance_by_vote( unsigned char* dest, const unsigned char* src, const unsigned char* neighbour, const unsigned char* background, unsigned int size )
// {
//     COMMA_THROW( comma::exception, "todo" )
// }

template < typename H >
advance< H >::advance( const parameters& param, bool a, int background ) // todo? quick and dirty: use cv::FilterEngine instead? (but cv::FilterEngine is so over-engineered)
    : background_( background )
    , set_pixel_( a ? &advance_by_nearest: &retreat_by_nearest )
{
    if( param.iterations > 1 ) { COMMA_THROW( comma::exception, "advance/retreat: only one iteration is supported; got: " << param.iterations ); }
    std::multimap< double, cv::Point > distances;
    cv::Point anchor( param.kernel.cols / 2, param.kernel.rows / 2 );    
    cv::Point p( 0, 0 );
    for( ; p.y < param.kernel.rows; ++p.y )
    {
        for( p.x = 0; p.x < param.kernel.cols; ++p.x )
        {
            if( p == anchor || !param.kernel.at< unsigned char >( p ) ) { continue; }
            cv::Point d = p - anchor;
            distances.insert( std::make_pair( d.x * d.x + d.y * d.y, d ) );
        }
    }
    offsets_.reserve( distances.size() );
    for( auto d: distances ) { offsets_.push_back( d.second ); }
}

template < typename H >
typename advance< H >::value_type advance< H >::operator()( value_type m )
{
    if( m.second.channels() > 1 ) { COMMA_THROW( comma::exception, "advance/retreat: got " << m.second.channels() << "-channel image; only 1-channel images supported now" ); }
    value_type n;
    n.first = m.first;
    m.second.copyTo( n.second );
    std::vector< unsigned char > background_pixel( m.second.elemSize() );
    for( unsigned int i = 0; i < m.second.elemSize(); i += m.second.elemSize1() ) { set_channel( &background_pixel[i], background_, m.second.depth() ); }
    tbb::parallel_for( tbb::blocked_range< int >( 0, m.second.rows, m.second.rows / 8 ), [&]( const tbb::blocked_range< int >& rows )
    {
        cv::Point p;
        for( p.y = rows.begin(); p.y < rows.end(); ++p.y )
        {
            for( p.x = 0; p.x < m.second.cols; ++p.x )
            {
                unsigned char* src = m.second.ptr( p.y, p.x );
                unsigned char* dest = n.second.ptr( p.y, p.x );
                for( unsigned int k = 0; k < offsets_.size(); ++k )
                {
                    cv::Point r = p + offsets_[k];
                    if( r.x < 0 || r.x >= m.second.cols || r.y < 0 || r.y >= m.second.rows ) { continue; }
                    unsigned char* neighbour = m.second.ptr( r.y, r.x );
                    if( set_pixel_( dest, src, neighbour, &background_pixel[0], background_pixel.size() ) ) { break; }
                }
            }
        }
    } );
    return n;
}

template < typename H >
advance< H > advance< H >::make( const std::vector< std::string >& options )
{
    int background = 0;
    std::vector< std::string > e = comma::split( options[1], ',' );
    if( e.back().substr( 0, 11 ) == "background:" ) // super quick and dirty
    {
        background = boost::lexical_cast< int >( e.back().substr( 11 ) );
        e.pop_back();
    }
    return advance< H >( parameters( std::vector< std::string >{{ options[0], comma::join( e, ',' ) }} ), options[0] == "advance", background ); // pain!
}

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::morphology::advance< boost::posix_time::ptime >;
template class snark::cv_mat::morphology::advance< std::vector< char > >;
template class snark::cv_mat::morphology::skeleton< boost::posix_time::ptime >;
template class snark::cv_mat::morphology::skeleton< std::vector< char > >;
