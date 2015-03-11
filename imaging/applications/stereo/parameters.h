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


#ifndef SNARK_IMAGING_APPLICATIONS_STEREO_PARAMETERS_H
#define SNARK_IMAGING_APPLICATIONS_STEREO_PARAMETERS_H

#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <comma/visiting/apply.h>
#include <comma/visiting/traits.h>

namespace snark { namespace imaging {

struct camera_parameters
{
    camera_parameters() : focal_length("0,0"),center("0,0"),distortion("0,0,0,0,0"),rotation("0,0,0"),translation("0,0,0"),size("0,0"){}

    std::string focal_length;
    std::string center;
    std::string distortion;
    std::string rotation;
    std::string translation;
    std::string size;
    std::string map;
};

/// parse camera parameters from config file
class camera_parser
{
public:
    typedef Eigen::Matrix< double, 5, 1 > Vector5d;

    camera_parser( const std::string& file, const std::string& path );

    const Eigen::Matrix3d& camera() const { return m_camera; }
    const Vector5d& distortion() const { return m_distortion; }
    const Eigen::Matrix3d& rotation() const { return m_rotation; }
    const Eigen::Vector3d& translation() const { return m_translation; }
    const cv::Mat& map_x() const { return m_map_x; }
    const cv::Mat& map_y() const { return m_map_y; }
    bool has_map() const { return ( m_map_x.cols != 0 ); }

private:
    Eigen::Matrix3d m_camera;
    Vector5d m_distortion;
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_translation;
    cv::Mat m_map_x;
    cv::Mat m_map_y;
};

} }

namespace comma { namespace visiting {

template <> struct traits< snark::imaging::camera_parameters >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::imaging::camera_parameters& c, Visitor& v )
    {
        v.apply( "focal-length", c.focal_length );
        v.apply( "centre", c.center );
        v.apply( "distortion", c.distortion );
        v.apply( "rotation", c.rotation );
        v.apply( "translation", c.translation );
        v.apply( "image-size", c.size );
        v.apply( "map", c.map );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::imaging::camera_parameters& c, Visitor& v )
    {
        v.apply( "focal-length", c.focal_length );
        v.apply( "centre", c.center );
        v.apply( "distortion", c.distortion );
        v.apply( "rotation", c.rotation );
        v.apply( "translation", c.translation );
        v.apply( "image-size", c.size );
        v.apply( "map", c.map );
    }
};

} }

#endif // SNARK_IMAGING_APPLICATIONS_STEREO_PARAMETERS_H
