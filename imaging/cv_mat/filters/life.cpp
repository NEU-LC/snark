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

/// @author vsevolod vlaskine

#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include "life.h"

namespace snark { namespace cv_mat { namespace filters {

template < typename H > life< H >::life( double procreation_treshold, double stability_threshold, double step, bool exit_on_stability )
    : procreation_treshold_( procreation_treshold )
    , stability_treshold_( stability_threshold )
    , step_( step )
    , exit_on_stability_( exit_on_stability )
{
}

static int next_( int i, int size ) { ++i; return i < size ? i : 0; }

static int prev_( int i, int size ) { return ( i > 0 ? i : size ) - 1; }

template < typename H > typename std::pair< H, cv::Mat > life< H >::operator()( typename std::pair< H, cv::Mat > p )
{
    bool changed = false;
    if( index_ )
    {
        cv::Mat current = generations_[ *index_ ].second;
        cv::Mat next = generations_[ 1 - *index_ ].second;
        tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, current.rows )
                         , [&]( const tbb::blocked_range< std::size_t >& r )
                         {
                             for( unsigned int i = r.begin(); i < r.end(); ++i )
                             {
                                 for( int j = 0; j < current.cols; ++j )
                                 {
                                     const unsigned char* c = current.ptr< unsigned char >( i, j );
                                     unsigned char* n = next.ptr< unsigned char >( i, j );
                                     for( int k = 0; k < current.channels(); ++k )
                                     {
                                         unsigned int sum = current.ptr< unsigned char >( prev_( i, current.rows ), prev_( j, current.cols ) )[k]
                                                          + current.ptr< unsigned char >( i,                        prev_( j, current.cols ) )[k]
                                                          + current.ptr< unsigned char >( next_( i, current.rows ), prev_( j, current.cols ) )[k]
                                                          + current.ptr< unsigned char >( prev_( i, current.rows ), j )[k]
                                                          + current.ptr< unsigned char >( next_( i, current.rows ), j )[k]
                                                          + current.ptr< unsigned char >( prev_( i, current.rows ), next_( j, current.cols ) )[k]
                                                          + current.ptr< unsigned char >( i,                        next_( j, current.cols ) )[k]
                                                          + current.ptr< unsigned char >( next_( i, current.rows ), next_( j, current.cols ) )[k];
                                         double normalized_sum = double( sum ) / 255; // quick and dirty
                                         double step = normalized_sum < procreation_treshold_ || normalized_sum > stability_treshold_ ? -step_
                                                     : ( normalized_sum - procreation_treshold_ ) < ( stability_treshold_ - normalized_sum ) ? step_
                                                     : 0;
                                         double new_value = c[k] + step * 255;
                                         n[k] = new_value < 0 ? 0 : new_value > 255 ? 255 : new_value;
                                         if( c[k] != n[k] ) { changed = true; }
                                     }
                                 }
                             }
                         } );
        index_ = 1 - *index_;
    }
    else
    {
        if( CV_MAT_DEPTH( p.second.type() ) != CV_8UC1 ) { COMMA_THROW( comma::exception, "life: only unsigned 8-bit images currently supported; it's life; got type: " << p.second.type() ); }
        index_ = 0;
        p.second.copyTo( generations_[0].second );
        p.second.copyTo( generations_[1].second );
        changed = true;
    }
    if( exit_on_stability_ && !changed ) { return std::make_pair( p.first, cv::Mat() ); }
    generations_[ *index_ ].first = p.first;
    return generations_[ *index_ ];
}

template class snark::cv_mat::filters::life< boost::posix_time::ptime >;
template class snark::cv_mat::filters::life< std::vector< char > >;

} } }  // namespace snark { namespace cv_mat { namespace impl {
