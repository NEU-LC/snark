// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_IMAGING_APPLICATIONS_STEREO_PARAMETERS_H
#define SNARK_IMAGING_APPLICATIONS_STEREO_PARAMETERS_H

#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <comma/visiting/apply.h>
#include <comma/visiting/traits.h>

namespace snark { namespace imaging {

struct camera_parameters
{
    std::string focal_length;
    std::string center;
    std::string distortion;
    std::string rotation;
    std::string translation;
    std::string size;
    std::string map_x;
    std::string map_y;
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
        v.apply( "map-x", c.map_x );
        v.apply( "map-y", c.map_y );
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
        v.apply( "map-x", c.map_x );
        v.apply( "map-y", c.map_y );
    }
};

} }

#endif // SNARK_IMAGING_APPLICATIONS_STEREO_PARAMETERS_H
