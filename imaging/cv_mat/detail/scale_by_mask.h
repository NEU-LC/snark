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

#pragma once

#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/pthread/pthread_mutex_scoped_lock.hpp>
#include "../filters.h"
#include "load-impl.h"

namespace snark{ namespace cv_mat {

// todo
// done - move implementation to a separate
// done - move as much as possible to the constructor
// done - get rid of unnecessary class members
// done - switch instead of if/else if or rather just don't use cvtColour
// done - dereference mask_ etc rather than using mask = mask_.get()
// done - make mask inflation thread-safe or don't inflate at all
// done - convert to mask type, multiply, convert back 

// template < typename H >
// struct scale_by_mask_ {
//     typedef typename impl::filters< H >::value_type value_type;
//     std::string mask_file_;
//     load_impl_< H > loader_;
//     boost::optional< cv::Mat > mask_;
//     
//     void apply_mask(cv::Mat& mat);
// 
//     scale_by_mask_( const std::string& mask_file ) : mask_file_(mask_file), loader_(mask_file) {}
// 
//     value_type operator()( value_type m );
// };
    
template < typename H >
struct scale_by_mask_ {
    typedef typename impl::filters< H >::value_type value_type;
    cv::Mat mask_;
    boost::shared_ptr< boost::mutex > mutex_;   // have to be careful here
    
    void apply_mask(cv::Mat& mat)
    {
        cv::Mat mat_float;
        mat.convertTo(mat_float, mask_.type());  // convert to float point
            
        cv::Mat masked = mat_float.mul(mask_);
        masked.convertTo(mat, mat.type() );       // convert back to 
    }

    scale_by_mask_( const std::string& mask_file ) : mutex_(new boost::mutex()) {
        mask_ = load_impl_< H >(mask_file)( value_type() ).second;
    }

    value_type operator()( value_type m )
    {
        cv::Mat& mat = m.second;
        
        if( mask_.depth() != CV_32FC1 && mask_.depth() != CV_64FC1 )  { COMMA_THROW(comma::exception, "failed scale-by-mask, mask type must be floating point f or  d"); }
        // We expand mask once, to match number of channels in input data
        if( mask_.channels() != mat.channels() )
        {
            boost::mutex::scoped_lock lock( *mutex_ );
            if( mask_.channels() != mat.channels() )    // Do a need an atomic to store mask_.channels() ?
            {
                if( mask_.channels() > 1 ) { COMMA_THROW(comma::exception, "mask channels (" << mask_.channels() << ")" <<" must be 1 or must be equal to input image channels: " << mat.channels()); }
                
                std::vector< cv::Mat > channels( mat.channels() );
                for( int i=0; i<mat.channels(); ++i ) { channels[i] = mask_; }
                cv::merge(channels, mask_);
            }
        }
        
        // For every input image we must check
        if( mat.rows != mask_.rows || mat.cols != mask_.cols ) { COMMA_THROW(comma::exception, "failed to apply scale-by-mask, because mask dimensions do not matches input row and column dimensions" ); }
        if( mask_.channels() != mat.channels() ) { COMMA_THROW(comma::exception, "mask_ file has more channels than input mat: " << mask_.channels() << " > " << mat.channels()); }
        
        if( mask_.depth() == mat.depth() ) { mat = mat.mul(mask_); }  // The simplest case
        else { apply_mask(mat); }
        
        return m;
    }
};

} }  // namespace snark { namespace cv_mat {
