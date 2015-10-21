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

#include <fstream>
#include <cstring>
#include <comma/base/exception.h>
#include "pinhole.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/test/utils/nullstream.hpp>

namespace snark { namespace camera {
    
boost::onullstream nullstream;
static std::ostream* cverbose=&nullstream;

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

Eigen::Vector2d pinhole::pixel_size() const { return Eigen::Vector2d( sensor_size.x() / image_size.x(), sensor_size.y() / image_size.y() ); }

static double squared_radius( const Eigen::Vector2d& p, const snark::camera::pinhole& c )
{
    return ( p - ( c.principal_point ? *c.principal_point : c.image_centre() ) ).squaredNorm();
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
static double extrapolate_map(const float* first, int width, double value)
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
    return tangentially_corrected( radially_corrected( p ) ); 
}

Eigen::Vector3d pinhole::to_cartesian( const Eigen::Vector2d& p, bool undistort ) const
{
    Eigen::Vector2d q = ( undistort ? undistorted( p ) : p ) - ( principal_point ? *principal_point : image_centre() );
    Eigen::Vector2d s = pixel_size();
    return Eigen::Vector3d( q.x() * s.x(), -q.y() * s.y(), -focal_length ); // todo: verify signs
}

Eigen::Vector2d pinhole::to_pixel( const Eigen::Vector3d& p, bool do_distort )
{
    Eigen::Vector2d s = pixel_size();
    Eigen::Vector2d q(p.x() / s.x(), -p.y() / s.y());
    q +=  principal_point ? *principal_point : image_centre();
    return do_distort ? distort(q) : q;
}

Eigen::Vector2d pinhole::image_centre() const { return Eigen::Vector2d( double( image_size.x() ) / 2, double( image_size.y() ) / 2 ); }

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
bool pinhole::distortion_t::all_zero() const { return radial.k1==0 && radial.k2==0 && radial.k3==0 && tangential.p1==0 && tangential.p2 == 0; }
Eigen::Vector2d pinhole::distort( const Eigen::Vector2d& p )
{
    if(!distortion.map)
    {
        *cverbose<<"pinhole::distort no map found"<<std::endl;
        if(distortion.all_zero()) { return p; }
        cv::Mat map_x;
        cv::Mat map_y;
        make_distortion_map(map_x,map_y);
        distortion.map=distortion_t::map_t(map_x,map_y);
        *cverbose<<"pinhole::distort made distortion map from parameters"<<std::endl;
    }
    Eigen::Vector2d dst;
    //lookup p.x on map.x
    int y_index=p.y()<0?0:(p.y()>image_size.y()?image_size.y()-1:p.y());
    double ox=extrapolate_map(&distortion.map->x_rows[y_index*image_size.x()],image_size.x(), p.x());
    int x_index=p.x()<0?0:(p.x()>image_size.x()?image_size.x()-1:p.x());
    double oy=extrapolate_map(&distortion.map->y_cols[x_index*image_size.y()], image_size.y(), p.y());
    return Eigen::Vector2d(ox,oy);
}
void pinhole::init(bool verbose)
{ 
    if(verbose)
        cverbose=&std::cerr;
    if (!distortion.map_filename.empty()) { distortion.map=load_distortion_map(); } 
    *cverbose<<"pinhole::init_distortion_map map:"<<distortion.map_filename<<" no map: "<< !distortion.map<<" image size "<< image_size.x()<<","<<image_size.y()<<std::endl;
    
}
//write a 2-d image matrix data to stream (no header)
inline void save(std::ostream& os, const cv::Mat& mat)
{
    std::streamsize len=(std::streamsize)mat.cols*mat.elemSize();
    for(int i=0;i<mat.rows;i++)
        std::cout.write((const char*)mat.ptr(i),len);
}
void pinhole::make_distortion_map(cv::Mat& map_x,cv::Mat& map_y) const
{
//     int width=image_size.x();
//     int heigth=image_size.y();
//     i:{0..w},j:{0..h} undistorted(i,j)
    cv::Mat camera=cv::Mat_<double>(3,3);
    camera.at<double>(0,0)=focal_length;
    camera.at<double>(1,1)=focal_length;
    Eigen::Vector2d c= principal_point ? *principal_point : image_centre();
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
void pinhole::output_distortion_map(std::ostream& os) const
{
    cv::Mat map_x;
    cv::Mat map_y;
    make_distortion_map(map_x,map_y);
     *cverbose<<"made distortion maps x.rows: "<<map_x.rows<<" x.cols: "<<map_x.cols<<" y.rows: "<<map_y.rows<<" y.cols: "<<map_y.cols<<std::endl;
    save(os, map_x);
    save(os, map_y);
}

} } // namespace snark { namespace camera {
