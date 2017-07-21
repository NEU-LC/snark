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

#include "morphology.h"

#include <string>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include <opencv2/imgproc/imgproc.hpp>

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

parameters::parameters(const std::vector< std::string >& e)
{
    if ( e.size() > 1 )
    {
        std::vector< std::string > p = comma::split( e[1], ',' );
        std::string eltype = p[0];
        try 
        {
            if ( eltype == "rectangle" || eltype == "ellipse" || eltype == "cross" ) 
            {
                if ( p.size() != 5 && p.size() != 3 && p.size() != 6 ) { COMMA_THROW( comma::exception, "structuring element of " << eltype << " type for the " << e[0] << " operation takes either 2, or 4 or 5 parameters" ); }
                while ( p.size() < 6 ) { p.push_back( "" ); }
                int size_x = ( p[1].empty() ? 3 : boost::lexical_cast< int >( p[1] ) );
                int size_y = ( p[2].empty() ? size_x : boost::lexical_cast< int >( p[2] ) );
                if ( size_x == 1 && size_y == 1 ) { std::cerr << "parse_structuring_element: warning: structuring element of a single point, no transformation is applied" << std::endl; }
                int anchor_x = ( p[3].empty() ? -1 : boost::lexical_cast< int >( p[3] ) );
                int anchor_y = ( p[4].empty() ? anchor_x : boost::lexical_cast< int >( p[4] ) );
                int shape = ( eltype == "rectangle" ? cv::MORPH_RECT : ( eltype == "ellipse" ? cv::MORPH_ELLIPSE : cv::MORPH_CROSS ) );
                kernel_ = cv::getStructuringElement( shape, cv::Size( size_x, size_y ), cv::Point( anchor_x, anchor_y ) );
                iterations_ = p[5].empty() ? 1 : boost::lexical_cast< comma::uint32 >(p[5]);
            } 
            else if ( eltype == "square" || eltype == "circle" ) 
            {
                if ( p.size() > 4 ) { COMMA_THROW( comma::exception, "structuring element of " << eltype << " type for the " << e[0] << " operation takes either 0, or 1, or 2, or 3 parameters" ); }
                while ( p.size() < 4 ) { p.push_back( "" ); }
                int size_x = ( p[1].empty() ? 3 : boost::lexical_cast< int >( p[1] ) );
                if ( size_x == 1 ) { std::cerr << "parse_structuring_element: warning: structuring element of a single point, no transformation is applied" << std::endl; }
                int anchor_x = ( p[2].empty() ? -1 : boost::lexical_cast< int >( p[2] ) );
                int shape = ( eltype == "square" ? cv::MORPH_RECT : cv::MORPH_ELLIPSE );
                kernel_ = cv::getStructuringElement( shape, cv::Size( size_x, size_x ), cv::Point( anchor_x, anchor_x ) );
                iterations_ = p[3].empty() ? 1 : boost::lexical_cast< comma::uint32 >(p[3]);
            } else {
                COMMA_THROW( comma::exception, "the '" << eltype << "' type of the structuring element is not one of rectangle,square,ellipse,circle,cross" );
            }
        } catch( boost::bad_lexical_cast blc ) { COMMA_THROW( comma::exception, "failed to cast parameter(s) for structuring element '" << eltype << "': " << blc.what() ); }
    }
    else { kernel_ = cv::Mat(); iterations_ = 1; }
}

template < typename H >
skeleton< H >::skeleton( const parameters& p ) : kernel_(p.kernel_), iterations_(p.iterations_) {}

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

} } }  // namespace snark { namespace cv_mat { namespace impl {

template class snark::cv_mat::morphology::skeleton< boost::posix_time::ptime >;
template class snark::cv_mat::morphology::skeleton< std::vector< char > >;
