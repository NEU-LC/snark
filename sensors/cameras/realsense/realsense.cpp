#include "realsense.h"
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <boost/lexical_cast.hpp>

namespace snark { namespace realsense {

namespace impl {
rs::format parse_format(const std::string& s)
{
    if(s.empty()) return rs::format::any;
    if(s=="any") return rs::format::any;
    if(s=="z16") return rs::format::z16;
    if(s=="disparity16") return rs::format::disparity16;
    if(s=="xyz32f") return rs::format::xyz32f;
    if(s=="yuyv") return rs::format::yuyv;
    if(s=="rgb8") return rs::format::rgb8;
    if(s=="bgr8") return rs::format::bgr8;
    if(s=="rgba8") return rs::format::rgba8;
    if(s=="bgra8") return rs::format::bgra8;
    if(s=="y8") return rs::format::y8;
    if(s=="y16") return rs::format::y16;
    if(s=="raw10") return rs::format::raw10;
    if(!s.empty()&&isdigit(s[0]))
    {
        int n=boost::lexical_cast<int>(s);
        if(n>=0&&n<=11) return (rs::format)n;
    }
    COMMA_THROW(comma::exception, "can't parse format: "<<s);
}

rs::stream parse_stream(const std::string& s)
{
    if(s=="depth") return rs::stream::depth;
    if(s=="color") return rs::stream::color;
    if(s=="infrared") return rs::stream::infrared;
    if(s=="infrared2") return rs::stream::infrared2;
    if(s=="points") return rs::stream::points;
    if(!s.empty()&&isdigit(s[0]))
    {
        int n=boost::lexical_cast<int>(s);
        if(n>=0&&n<=10) return (rs::stream)n;
    }
    COMMA_THROW( comma::exception, "invalid stream name: "<<s);
}

}   //namespace impl {

format_t::format_t():value(rs::format::any) { }
format_t::format_t(const std::string& s):value(impl::parse_format(s)) { }
format_t::format_t(rs::format f):value(f) { }
format_t::operator rs::format() { return value; }
unsigned format_t::cv_type() const
{
    switch(value)
    {
        case rs::format::z16:
        case rs::format::disparity16:
        case rs::format::y16:
            return CV_16UC1;
        case rs::format::xyz32f:
            return CV_32FC3;
        case rs::format::y8:
            return CV_8UC1;
        case rs::format::rgb8: 
        case rs::format::bgr8:
            return CV_8UC3;
        case rs::format::rgba8:
        case rs::format::bgra8:
        case rs::format::yuyv:
            return CV_8UC4;
        case rs::format::raw10: //5 bytes format, doesn't fit any cv types; but CV_MAKETYPE(CV_8U,5) would have same size
        default:
        { COMMA_THROW( comma::exception, "format not supported: "<<value); }
    }
}
size_t format_t::size() const
{
    switch(value)
    {
        case rs::format::z16: 
        case rs::format::disparity16:
            return sizeof(std::int16_t);
        case rs::format::xyz32f:
            return 3*sizeof(std::int32_t); //32-bit float
        case rs::format::rgb8: 
        case rs::format::bgr8:
            return 3*sizeof(std::int8_t);
        case rs::format::rgba8:
        case rs::format::bgra8:
            return 4*sizeof(std::int8_t);
        case rs::format::raw10:
            return 5;
        //not sure?
        case rs::format::y8:
            return sizeof(std::int8_t);
        case rs::format::y16:
            return sizeof(std::int16_t);
        //??
        case rs::format::yuyv:
            return 4*sizeof(std::int8_t);
        default:
        { COMMA_THROW(comma::exception, "unknown format size: "<<value); }
    }
}

/*********************************************************/
stream_t::stream_t(rs::device& device, const std::string& s,format_t f) : device(device), value(impl::parse_stream(s))
{
    init(f);
}
stream_t::stream_t(rs::device& device, rs::stream id,format_t f) : device(device), value(id)
{
    init(f);
}
void stream_t::init(format_t f)
{
    device.enable_stream(value,0,0,f,0);
    format=device.get_stream_format(value);
    width=device.get_stream_width(value);
    height=device.get_stream_height(value);
    //size=width*height*format.size();
    comma::verbose<<"stream_writer: stream "<<value<<" width "<<width<<" height "<<height<<" format "<<format<<std::endl;
}
std::pair<boost::posix_time::ptime,cv::Mat> stream_t::get_frame()
{
    boost::posix_time::ptime time=start_time+boost::posix_time::milliseconds(device.get_frame_timestamp(value));
    cv::Mat mat(height,width,format.cv_type(),(void*)(device.get_frame_data(value)));
    return std::pair<boost::posix_time::ptime,cv::Mat>(time,mat);
}
/*********************************************************/
points_cloud::points_cloud(rs::device& device) : device(device)
{
    device.enable_stream(rs::stream::depth,0,0,rs::format::any,0);
}
void points_cloud::scan()
{
    if(sizeof(points[0])!=4) { COMMA_THROW( comma::exception, "expected float to be 32-bit, got"<<sizeof(points[0])); }
    points=static_cast<const float*>(device.get_frame_data(rs::stream::points));
}
Eigen::Vector3d points_cloud::get(int i)
{
    i*=3;
    return Eigen::Vector3d(points[i],points[i+1],points[i+2]);
}
Eigen::Vector2i points_cloud::project(int i)
{
    return Eigen::Vector2i(i,0);
}
int points_cloud::count()
{
    return 10;
}
/*********************************************************/
run_stream::run_stream(rs::device& d):device(d) 
{ 
    device.start(); 
    start_time=boost::posix_time::microsec_clock::universal_time();
}
run_stream::~run_stream() { device.stop(); }

} } //namespace snark { namespace realsense {
