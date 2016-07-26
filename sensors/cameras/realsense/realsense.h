#pragma once
#include <librealsense/rs.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Eigen/Core>

namespace snark { namespace realsense {

struct camera
{
    rs::device& device;
    camera(rs::device& device);
};

//pixel format helper
struct format_t
{
    rs::format value;
    format_t();
    format_t(const std::string& s);
    format_t(rs::format f);
    unsigned cv_type() const; //e.g. CV_8UC3
    size_t size() const;  //size in bytes
    operator rs::format();
};

//individual camera stream
struct stream_t
{
    rs::device& device;
    rs::stream value;
    format_t format;
    boost::posix_time::ptime start_time;
    stream_t(rs::device& device, rs::stream id,format_t format=format_t());
    stream_t(rs::device& device, const std::string& s,format_t format=format_t());
    std::pair<boost::posix_time::ptime,cv::Mat> get_frame();
private:
    //size_t size;
    int width;
    int height;
    void init(format_t format);
};

struct points_cloud
{
    points_cloud(rs::device& device);
    Eigen::Vector3d get(int i);
    Eigen::Vector2i project(int i);
    int count();
    void scan();
    rs::device& device;
    const float* points;
};

struct run_stream
{
    rs::device& device;
    boost::posix_time::ptime start_time;
    run_stream(rs::device& d);
    ~run_stream();
};

inline std::ostream& operator<<(std::ostream& o,format_t format) { return o << format.value; }

} } //namespace snark { namespace realsense {
