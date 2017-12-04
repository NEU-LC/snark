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

#pragma once

#include "cepton_sdk.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>

namespace snark { namespace cepton {

/// lidar point for cepton
struct point_t
{
    boost::posix_time::ptime t;
    uint32_t block;
    Eigen::Vector3f point;
    float intensity;
    point_t( ) : intensity(0) { }
    point_t(boost::posix_time::ptime t,const CeptonSensorPoint& cp,uint32_t block=0);
};



/// only one instance of this class can be constructed at any time
class device
{
public:
    device( boost::optional< uint16_t > port
          , bool disable_image_clip=false
          , bool disable_distance_clip=false);
    ~device();
    
    // cepton device information as iterator
    struct iterator
    {
        iterator(int x=0);
        int index;
        CeptonSensorInformation const * operator*() const;
        void operator++(int);
        bool operator<(const iterator& rhs) const;
    };
    iterator begin();
    iterator end();
    
    //frame listener
    struct listener
    {
        listener(unsigned frames_per_block=1,bool system_time=true);
        ~listener();
        virtual void on_frame(const std::vector<point_t>& points) = 0;
        uint32_t block;
        unsigned frames_per_block;
        bool system_time;
    private:
        static void on_frame(int error_code, CeptonSensorHandle sensor, size_t n_points, struct CeptonSensorPoint const *p_points);
        static listener* instance_;
    };
    
private:
    static void on_event(int error_code, CeptonSensorHandle sensor, struct CeptonSensorInformation const *p_info, int event);
    static device* instance_;
    bool attached_;
};
    
} } //namespace snark { namespace cepton {
    
