// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace cv_mat { namespace filters {

template < typename H >
struct hard_edge
{
    static std::pair< H, cv::Mat > handle( std::pair< H, cv::Mat > m, float background = 0 );
    
    typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;
    
    /// return hard edge functor and boolean flag that indicates whether functor is safely re-entrant
    /// in multithread context, which is always true, since the functor always is re-entrant
    static std::pair< functor_t, bool > make( const std::string& option );
    
    static std::string usage( unsigned int indent = 0 );        
};

} } }  // namespace snark { namespace cv_mat { namespace impl {
