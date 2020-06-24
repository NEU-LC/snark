// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/base/types.h>
#include <opencv2/core/core.hpp>

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
class convolution
{
    public:
        typedef std::pair< H, cv::Mat > value_type;

        convolution( cv::Mat kernel, int ddepth = -1, const cv::Point& anchor = cv::Point( -1, -1 ), double delta = 0, int border = cv::BORDER_DEFAULT );

        value_type operator()( value_type );
        
        static convolution make( const std::vector< std::string >& options );
        
        static std::string usage( unsigned int indent = 0 );
        
    private:
        cv::Mat kernel_;
        int ddepth_;
        cv::Point anchor_;
        double delta_;
        int border_;
};

} } } // namespace snark { namespace cv_mat { namespace filters {
