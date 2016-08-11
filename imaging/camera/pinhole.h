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

/// @authors vsevolod vlaskine, zhe xu

#ifndef SNARK_IMAGING_CAMERA_PINHOLE_H
#define SNARK_IMAGING_CAMERA_PINHOLE_H

#include <vector>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <comma/sync/lazy.h>

namespace snark { namespace camera {

class pinhole
{
    public:
        struct config_t
        {
            struct distortion_t
            {
                struct radial_t
                {
                    double k1, k2, k3;
                    
                    radial_t() : k1( 0 ), k2( 0 ), k3( 0 ) {}
                    
                    bool empty() const;
                };
                
                struct tangential_t
                {
                    double p1, p2;
                    
                    tangential_t() : p1( 0 ), p2( 0 ) {}
                    
                    bool empty() const;
                };
                
                radial_t radial;
                
                tangential_t tangential;
                
                std::string map_filename;
                
                bool empty() const;
                                
                template < typename V > V as() const;
            };
            
            /// focal length in metres (if sensor_size is empty then focal length is effectively in pixels)
            double focal_length;
            //quick and dirty: if sensor_size is empty we are taking pixel size to be 1 meter !?

            /// sensor size in metres
            boost::optional< Eigen::Vector2d > sensor_size;
            
            /// image size in pixels
            Eigen::Vector2i image_size;
            
            /// principal point in pixels; if not defined, then image_size / 2
            boost::optional< Eigen::Vector2d > principal_point;
            
            /// distortion
            boost::optional< distortion_t > distortion;
            
            /// default constructor
            config_t();
            
            /// return pixel size in metres or 1,1 if sensor_size is empty
            Eigen::Vector2d pixel_size() const;
            
            /// return principal_point or if empty returns half image size in pixels
            Eigen::Vector2d image_centre() const;
            
            /// return radially corrected pixel
            Eigen::Vector2d radially_corrected( const Eigen::Vector2d& p ) const;
            
            /// return tangentially corrected pixel
            Eigen::Vector2d tangentially_corrected( const Eigen::Vector2d& p ) const;
            
            /// throw, if basic checks on config fail
            void validate();
        };
        
        pinhole( const pinhole::config_t& config );
        
        /// return pixel coordinates in camera frame
        Eigen::Vector3d to_cartesian( const Eigen::Vector2d& p, bool undistort = true ) const;
        Eigen::Vector3d to_cartesian_deprecated( const Eigen::Vector2d& p, bool undistort = true ) const;

        //returns converts from camera frame to image pixel col,row
        Eigen::Vector2d to_pixel( const Eigen::Vector2d& p ) const;
        Eigen::Vector2d to_pixel_deprecated( const Eigen::Vector2d& p ) const;
        
        /// return radially and then tangentially corrected pixel
        Eigen::Vector2d undistorted( const Eigen::Vector2d& p ) const;
        
        /// reverse undistorted projection using the projection map
        Eigen::Vector2d distort( const Eigen::Vector2d& p ) const;
        
        const pinhole::config_t& config() const { return config_; } 
        
        static std::string usage();
        
        class distortion_map_t
        {
            public:
                cv::Mat map_x;
                
                cv::Mat map_y;
                
                distortion_map_t( const pinhole::config_t& config );
            
                void write( std::ostream& os ) const;
                
            private:
                friend class pinhole; // quick and dirty
                std::vector< float > x_rows_; // quick and dirty, hide for now, refactor later; rows of undistort map x
                std::vector< float > y_cols_; // quick and dirty, hide for now, refactor later; transposed columns of undistort map y
        };
        
        const boost::optional< distortion_map_t >& distortion_map() const;
        
    private:
        pinhole::config_t config_;
        Eigen::Vector2d pixel_size_;
        Eigen::Vector2d image_centre_;
        comma::lazy< boost::optional< distortion_map_t > > distortion_map_; // since it's a slow operation, initialize on demand
};

} } // namespace snark { namespace camera {

#endif // SNARK_IMAGING_CAMERA_PINHOLE_H
