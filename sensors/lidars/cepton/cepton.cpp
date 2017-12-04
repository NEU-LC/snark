// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

#include "cepton.h"
#include <comma/base/exception.h>
#include <comma/application/verbose.h>

namespace snark { namespace cepton {

point_t::point_t(boost::posix_time::ptime t,const CeptonSensorPoint& cp,uint32_t block) : 
    t(t), 
    block(block),
    point(cp.x,cp.y,cp.z), 
    intensity(cp.intensity)
{
    
} 
    
device* device::instance_=NULL;
device::listener* device::listener::instance_=NULL;
    
void device::on_event(int error_code, CeptonSensorHandle sensor, struct CeptonSensorInformation const *p_info, int event)
{
    switch(event)
    {
        case CEPTON_EVENT_ATTACH:
            if(instance_)
                instance_->attached_=true;
            break;
        // not used?
        case CEPTON_EVENT_DETACH:
            if(instance_)
                instance_->attached_=false;
            break;
    }
    comma::verbose<<"cepton on_event "<<event<<" "<<error_code<<std::endl;
}

device::device( boost::optional< uint16_t > port, bool disable_image_clip, bool disable_distance_clip )
    : attached_( false )
{
    int err;
    if(instance_) { COMMA_THROW( comma::exception, "only one instance of cepton::device can be constructed"); }
    instance_=this;

    // By default the sdk listens on ports 8808 and 2368, however the device
    // only transmits on a single port (typically 8808). To avoid clashes with
    // other devices (e.g. the Velodyne Puck which transmits on port 2368) use
    // the --port option to open only the required port.
    if( port )
    {
        comma::verbose << "listening on port " << *port << std::endl;
        err = cepton_sdk_set_ports( &*port, 1 );
        if( err != CEPTON_SUCCESS ) { COMMA_THROW( comma::exception, "cepton_sdk_set_ports failed: " << cepton_get_error_code_name( err )); }
    }

    uint32_t control_flags= 
        (disable_image_clip ? CEPTON_SDK_CONTROL_DISABLE_IMAGE_CLIP : 0) | 
        (disable_distance_clip ? CEPTON_SDK_CONTROL_DISABLE_DISTANCE_CLIP : 0);
    err=cepton_sdk_initialize(CEPTON_SDK_VERSION,control_flags,&device::on_event);
    if (err != CEPTON_SUCCESS) { COMMA_THROW(comma::exception,"cepton_sdk_initialize failed: "<<cepton_get_error_code_name(err)); }
    comma::verbose<<"cepton init "<<err<<std::endl;
    for(int i=0;i<40&&!attached_;i++)
    {
        usleep(500000);
    }
}
    
device::~device()
{
    cepton_sdk_deinitialize();
    instance_=NULL;
}
device::iterator device::begin()
{
    return device::iterator(0);
}
device::iterator device::end()
{
    auto res=cepton_sdk_get_number_of_sensors();
    comma::verbose<<"cepton count "<<res<<std::endl;
    return device::iterator(res);
}
device::iterator::iterator(int x) : index(x) { }
CeptonSensorInformation const * device::iterator::operator*() const
{
    int count=cepton_sdk_get_number_of_sensors();
    if(index<0||index>=count) { COMMA_THROW(comma::exception, "index ("<<index<<") out of range 0-"<<count );}
    return cepton_sdk_get_sensor_information_by_index(index);
}
void device::iterator::operator++(int) { index++; }
bool device::iterator::operator<(const iterator& rhs) const { return index<rhs.index; }

device::listener::listener(unsigned frames_per_block,bool system_time) : block(0),frames_per_block(frames_per_block),system_time(system_time)
{
    if(instance_) { COMMA_THROW( comma::exception,"only one listener allowed at a time"); }
    instance_=this;
    int err=cepton_sdk_listen_frames(&device::listener::on_frame);
    if (err != CEPTON_SUCCESS) { COMMA_THROW(comma::exception,"cepton_sdk_listen_frames failed: "<<cepton_get_error_code_name(err)); }
}
device::listener::~listener()
{
    instance_=NULL;
    int err=cepton_sdk_unlisten_frames(&device::listener::on_frame);
    if (err != CEPTON_SUCCESS) { COMMA_THROW(comma::exception,"cepton_sdk_unlisten_frames failed: "<<cepton_get_error_code_name(err)); }
}
void device::listener::on_frame(int error_code, CeptonSensorHandle sensor, size_t n_points, struct CeptonSensorPoint const *p_points)
{
    boost::posix_time::ptime t0=boost::posix_time::microsec_clock::universal_time();
    if(!instance_)
        return;
    if(!n_points)
        return;
    std::vector<point_t> points(n_points);
    uint32_t block=instance_->block/instance_->frames_per_block;
    uint64_t timestamp0=p_points[0].timestamp;
    if(!instance_->system_time)
    {
        t0=boost::posix_time::ptime(boost::gregorian::date( 1970, 1, 1 ),boost::posix_time::microseconds(0));
        timestamp0=0;
    }
    for(unsigned i=0;i<n_points;i++)
    {
        points[i]=point_t(t0+boost::posix_time::microseconds(p_points[i].timestamp-timestamp0),p_points[i],block);
    }
    instance_->on_frame(points);
    instance_->block++;
}
    
} } //namespace snark { namespace cepton {
    
