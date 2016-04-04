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

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include "pinhole.h"

namespace snark { namespace camera {
    
//sensor_size and principal_point are optional (set to empty)
pinhole::pinhole() : focal_length( 0 ), image_size( Eigen::Vector2i::Zero() ) {}
pinhole::distortion_t::operator Eigen::Matrix< double, 5, 1 >() const
{
    Eigen::Matrix< double, 5, 1 > m;
    m[0] = radial.k1;
    m[1] = radial.k2;
    m[2] = tangential.p1;
    m[3] = tangential.p2;
    m[4] = radial.k3;
    return m;
}

//quick and dirty: if sensor_size is empty we are taking pixel size to be 1 meter !?
Eigen::Vector2d pinhole::pixel_size() const { return sensor_size ? Eigen::Vector2d( sensor_size->x() / image_size.x(), sensor_size->y() / image_size.y() ) : Eigen::Vector2d(1,1); }

static double squared_radius( const Eigen::Vector2d& p, const snark::camera::pinhole& c )
{
    return ( p - c.image_centre() ).squaredNorm();
}

Eigen::Vector2d pinhole::radially_corrected( const Eigen::Vector2d& p ) const
{
    double r2 = squared_radius( p, *this );
    double k = 1 + distortion.radial.k1 * r2 + distortion.radial.k2 * r2 * r2 + distortion.radial.k3 * r2 * r2 * r2;
    return p * k;
}

Eigen::Vector2d pinhole::tangentially_corrected( const Eigen::Vector2d& p ) const
{
    double r2 = squared_radius( p, *this );
    double xy = p.x() * p.y();
    return p + Eigen::Vector2d( distortion.tangential.p1 * 2 * xy + distortion.tangential.p2 * ( r2 + p.x() * p.x() * 2 )
                              , distortion.tangential.p2 * 2 * xy + distortion.tangential.p1 * ( r2 + p.y() * p.y() * 2 ) );
}
static double reverse_map(std::vector<float>::const_iterator first, unsigned int width, double value)
{
    std::vector<float>::const_iterator last=first + width;
    std::vector<float>::const_iterator right=std::upper_bound(first,last, value);
    if(right == last)
        right--;
    if(right == first)
        right++;
    std::vector<float>::const_iterator left = right - 1;
    //std::cerr<<"extrapolate_reverse_map: left [" <<int(left - first)<<"]: " << *left<< " right ["<<int(right - first)<<"]:"<<*right<<" /"<<int(last-first)<<" value "<<value<<std::endl;
    double o=double(value - *left) / double(*right - *left) + int(left - first);
    //std::cerr<<"a "<<(value - *left)<<" b "<<(*right - *left)<<" -> "<<o<<std::endl;
    return o;
}
static double extrapolate_map(const float* first, unsigned int width, double value)
{
    const float* left=first + int(value);
    if(value<0)
        left=first;
    else
    {
        if(value>=width-1)
            left=first+width-2;
    }
    const float* right=left+1;
    double frac=value - (left - first);
    double step = *right - *left;
    return double(frac * step + *left);
}
Eigen::Vector2d pinhole::undistorted( const Eigen::Vector2d& p ) const
{
    if( !const_cast< pinhole* >( this )->make_distortion_map_impl_() ) { return p; } // quick and dirty
    //lookup the map to undistort coordinates
    int y_index= (p.y()<0) ? 0 : (p.y()>image_size.y()?image_size.y()-1:p.y());
    double ox=reverse_map(distortion.map->x_rows.begin() + y_index*image_size.x(), (unsigned int)image_size.x(), p.x());
    int x_index= (p.x()<0) ? 0 : (p.x()>image_size.x()?image_size.x()-1:p.x());
    double oy=reverse_map(distortion.map->y_cols.begin() + x_index*image_size.y(), (unsigned int)image_size.y(), p.y());
    return Eigen::Vector2d(ox,oy);
}
//Eigen::Vector2d pinhole::undistorted( const Eigen::Vector2d& p ) const
Eigen::Vector2d pinhole_to_distorted(const pinhole& pinhole, const Eigen::Vector2d& p ) //const
{
    return pinhole.tangentially_corrected( pinhole.radially_corrected( p ) ); 
}

// todo: temporarily parametrize on the frame required
Eigen::Vector3d pinhole::to_cartesian( const Eigen::Vector2d& p, bool undistort ) const
{
    Eigen::Vector2d q = ( undistort ? undistorted( p ) : p ) - image_centre();
    Eigen::Vector2d s = pixel_size();
    return Eigen::Vector3d( -q.x() * s.x(), -q.y() * s.y(), -focal_length );
}

Eigen::Vector2d pinhole::to_pixel( const Eigen::Vector2d& p ) const
{
    Eigen::Vector2d s = pixel_size();
    return Eigen::Vector2d( -p.x() / s.x(), -p.y() / s.y() ) + image_centre();
}

Eigen::Vector3d pinhole::to_cartesian_deprecated( const Eigen::Vector2d& p, bool undistort ) const
{
    Eigen::Vector2d q = ( undistort ? undistorted( p ) : p ) - image_centre();
    Eigen::Vector2d s = pixel_size();
    return Eigen::Vector3d( q.x() * s.x(), -q.y() * s.y(), -focal_length ); // todo: verify signs
}

Eigen::Vector2d pinhole::to_pixel_deprecated( const Eigen::Vector2d& p ) const
{
    Eigen::Vector2d s = pixel_size();
    Eigen::Vector2d q( p.x() / s.x(), p.y() / s.y() );
    q += image_centre();
    return q;
}

Eigen::Vector2d pinhole::image_centre() const { return principal_point ? *principal_point : Eigen::Vector2d( double( image_size.x() ) / 2, double( image_size.y() ) / 2 ); }

static void load( const std::string& filename, const Eigen::Vector2i& image_size, cv::Mat& map_x, cv::Mat& map_y )
{
    if( filename.empty() ) { COMMA_THROW( comma::exception, "distortion map filename not specified" ); }
    std::ifstream ifs( &filename[0], std::ios::binary );
    if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open " << filename ); }
    std::vector< char > buffer( image_size.x() * image_size.y() * 4 );
    ifs.read( &buffer[0], buffer.size() );
    if( ifs.gcount() < int( buffer.size() ) ) { COMMA_THROW( comma::exception, "expected to read " << buffer.size() << " bytes, got " << ifs.gcount() ); }
    map_x = cv::Mat( image_size.y(), image_size.x(), CV_32FC1, &buffer[0] ).clone();
    ifs.read( &buffer[0], buffer.size() );
    if( ifs.gcount() < int( buffer.size() ) ) { COMMA_THROW( comma::exception, "expected to read " << buffer.size() << " bytes, got " << ifs.gcount() ); }
    map_y = cv::Mat( image_size.y(), image_size.x(), CV_32FC1, &buffer[0] ).clone();
}
pinhole::distortion_t::map_t::map_t(const cv::Mat& map_x, const cv::Mat& map_y)
{
    x_rows.resize(map_x.rows*map_x.cols);
    for(int i=0;i<map_x.rows;i++)
    {
        cv::Mat mat_row=map_x.row(i);
        float* ptr=mat_row.ptr<float>();
        std::memcpy(&x_rows[map_x.cols*i],ptr,map_x.cols*sizeof(float));
    }
    //
    y_cols.resize(map_y.rows*map_y.cols);
    for(int i=0;i<map_y.cols;i++)
    {
        cv::Mat mat_col=map_y.col(i).t();
        float* ptr=mat_col.ptr<float>();
        std::memcpy(&y_cols[map_y.rows*i],ptr,map_y.rows*sizeof(float));
    }
}
pinhole::distortion_t::map_t pinhole::load_distortion_map() const 
{
    cv::Mat map_x;
    cv::Mat map_y;
    load(distortion.map_filename, image_size, map_x,map_y);
    return pinhole::distortion_t::map_t(map_x,map_y);
}
bool pinhole::make_distortion_map_impl_()
{
    if( distortion.map ) { return true; }
    if( distortion.all_zero() ) { return false; }
    cv::Mat map_x;
    cv::Mat map_y;
    make_distortion_map( map_x, map_y );
    distortion.map = distortion_t::map_t( map_x, map_y ); // quick and uber-dirty; not proud of it, not proud
    return true;
}
bool pinhole::distortion_t::all_zero() const { return radial.k1==0 && radial.k2==0 && radial.k3==0 && tangential.p1==0 && tangential.p2 == 0; }

Eigen::Vector2d pinhole::distort( const Eigen::Vector2d& p ) const
{
    if( !const_cast< pinhole* >( this )->make_distortion_map_impl_() ) { return p; } // quick and dirty
    //lookup p.x on map.x
    int y_index= (p.y()<0) ? 0 : (p.y()>image_size.y()?image_size.y()-1:p.y());
    double ox=extrapolate_map(&distortion.map->x_rows[y_index*image_size.x()], (unsigned int)image_size.x(), p.x());
    int x_index= (p.x()<0) ? 0 : (p.x()>image_size.x()?image_size.x()-1:p.x());
    double oy=extrapolate_map(&distortion.map->y_cols[x_index*image_size.y()], (unsigned int)image_size.y(), p.y());
    return Eigen::Vector2d(ox,oy);
}
void pinhole::init()
{ 
    if( !distortion.map_filename.empty() ) { distortion.map=load_distortion_map(); }
    comma::verbose<<"pinhole::init_distortion_map map:"<<distortion.map_filename<<" no map: "<< !distortion.map<<" image size "<< image_size.x()<<","<<image_size.y()<<std::endl;
    
}
//write a 2-d image matrix data to stream (no header)
static void save( std::ostream& os, const cv::Mat& mat )
{
    std::streamsize len=(std::streamsize)mat.cols*mat.elemSize();
    for(int i=0;i<mat.rows;i++) { os.write((const char*)mat.ptr(i),len); }
}

void pinhole::make_distortion_map(cv::Mat& map_x,cv::Mat& map_y) const
{
//     int width=image_size.x();
//     int heigth=image_size.y();
//     i:{0..w},j:{0..h} undistorted(i,j)
    cv::Mat camera=cv::Mat_<double>(3,3);
    camera.at< double >( 0, 0 ) = focal_length * ( sensor_size ? double( image_size.x() ) / sensor_size->x() : 1.0 );
    camera.at< double >( 1, 1 ) = focal_length * ( sensor_size ? double( image_size.y() ) / sensor_size->y() : 1.0 );
    Eigen::Vector2d c= image_centre();
    camera.at<double>(0,2)=c.x();
    camera.at<double>(1,2)=c.y();
    camera.at<double>(2,2)=1;
    cv::Vec<double,5> distortion_coeff;
    distortion_coeff[0]=distortion.radial.k1;
    distortion_coeff[1]=distortion.radial.k2;
    distortion_coeff[2]=distortion.tangential.p1;
    distortion_coeff[3]=distortion.tangential.p2;
    distortion_coeff[4]=distortion.radial.k3;
    cv::Size size(image_size.x(),image_size.y());
    cv::initUndistortRectifyMap(camera, distortion_coeff,cv::Mat(),camera,size,CV_32FC1,map_x,map_y);
}

void pinhole::output_distortion_map( std::ostream& os ) const
{
    cv::Mat map_x;
    cv::Mat map_y;
    make_distortion_map( map_x, map_y );
    save( os, map_x );
    save( os, map_y );
}

std::string pinhole::usage()
{
    std::ostringstream oss;
    oss << "camera config: for a sample, try image-pinhole --output-config" << std::endl;
    oss << "    sensor_size: sensor dimensions; optional; if absent, focal_length should be effectively defined in pixels" << std::endl;
    oss << "        sensor_size/x: sensor width, meters" << std::endl;
    oss << "        sensor_size/y: sensor height, m, meters" << std::endl;
    oss << "    image_size: image dimensions, pixels" << std::endl;
    oss << "        image_size/x: image width, pixels" << std::endl;
    oss << "        image_size/y: image height, pixels" << std::endl;
    oss << "    focal_length: focal length, meters" << std::endl;
    oss << "    principal_point: image centre, pixels, default: geometrical image centre" << std::endl;
    oss << "        principal_point/x" << std::endl;
    oss << "        principal_point/y" << std::endl;
    oss << "    distortion: optional" << std::endl;
    oss << "        distortion/radial/k1" << std::endl;
    oss << "        distortion/radial/k2" << std::endl;
    oss << "        distortion/radial/k3" << std::endl;
    oss << "        distortion/tangential/p1" << std::endl;
    oss << "        distortion/tangential/p2" << std::endl;
    oss << "        distortion/map=<filename>: distortion map as two concatenated binary distortion maps for x and y" << std::endl;
    oss << "                                   each map is matrix of floats of size image_size/x X image_size/y" << std::endl;
    oss << "    calculated fields (don't specify in config):" << std::endl;
    oss << "        pixel size: calculated as sensor_size/image_size; this is used to convert between cartesian frame (meter) and image frame (pixels)" << std::endl;
    oss << "            if sensor_size is empty (not specified) then pixel_size is assumed to be 1 meter; which means focal_length is effectively in pixels and no scaling when converting between cartesian/image frame" << std::endl;
    return oss.str();
}

} } // namespace snark { namespace camera {
