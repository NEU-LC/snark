#pragma once

#include "device.h"
#include "../message.h"

namespace snark { namespace hokuyo {

struct scip2_data
{
    //MD: 3 bytes
    typedef comma::packed::scip_3chars_t single_t;
    char* buf;
    std::size_t buf_size; //size of buf
    char* data;
    int count_steps;    //count of data array
    //request param
    int start_step;
    
    scip2_data();
    ~scip2_data();
    void set_param(int start, int steps);    //allocate enough buffer; clear current data if necessary
    void unpack();
    comma::uint32 get(int i);
    void clear();
};

struct scip2_profile
{
    int min_step;
    int max_step;
    int slit_division;  //number of steps in full circle
    int sensor_front_step;
    double min_distance;
    double step_to_bearing(int step);
    virtual void range_check(int start_step, int end_step);   //throw exception if invalid range
    scip2_profile():min_step(0),max_step(0),slit_division(1),sensor_front_step(0),min_distance(0.000001) { }
};
struct urg_04lx : public scip2_profile
{ 
    urg_04lx(); 
    virtual void range_check(int start_step, int end_step);
};
struct uhg_08lx : public scip2_profile
{ 
    uhg_08lx(); 
};
struct utm_30lx : public scip2_profile
{ 
    utm_30lx(); 
};


struct scip2_device:public laser_device
{
    static urg_04lx urg_04lx_;
    static uhg_08lx uhg_08lx_;
    static utm_30lx utm_30lx_;
    scip2_data scan_data;
    boost::shared_ptr<request_md> md;
    struct data_t
    {
        double distance;
        double bearing;
        data_t():distance(0),bearing(0){}
        data_t(double d, double b):distance(d),bearing(b){}
    };
    /// Represent a point relative to the laser's coordinate frame.
    /// Note z and elevation are always zero as laser shoots out horizontally.
    struct output_t
    {
        /// distance in meters, bearing in (radians)
        output_t();
        output_t(double distance, double bearing);
        bool is_nan() const { return ( x == 0 && y == 0 && z == 0 ); }
        
        boost::posix_time::ptime t;
        double x;
        double y;
        double z;
        unsigned int block;
        double range; // meters
        double bearing; //radians
        double elevation;   // radians
    };
    scip2_profile* profile; //typically points to one of static members above
    //options
    int baud_rate;
    int set_baud_rate;
    std::string port;
    scip2_device():profile(&urg_04lx_), baud_rate(0)
    {
    }
    virtual void init(comma::command_line_options options)
    {
        //get args
        baud_rate=options.value<int>("--baud-rate", 0);
        port=options.value<std::string>("--serial,--port");
        set_baud_rate=options.value<int>("--set-baud-rate", 500000);
    }
    virtual boost::shared_ptr<stream_base> connect();
    //e.g. switch to scip2
    virtual void setup(stream_base& ios);
    virtual void reboot(stream_base& ios) { }    //doesn't support reboot
    //turn_laser_on/off or use common method
    //start receiving data
    void request_scan(stream_base& ios, int start_step, int end_step, int num_of_scans);
    //return false if num_of_scans in response == 0
    bool receive_response(stream_base& ios);
    //use after response is received
    data_t get_data(int scan);
    //is data valid
    bool is_valid(data_t& data) const;
    output_t convert(data_t& data);
    int get_steps() const;
};

} }  // namespace snark { namespace hokuyo {
