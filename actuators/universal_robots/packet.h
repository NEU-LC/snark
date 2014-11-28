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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#include <boost/array.hpp>
#include <comma/packed/packed.h>
#include "arm.h"

namespace comma { namespace ur {

struct packet_t : public comma::packed::packed_struct< packet_t, 812  >
{
    comma::packed::net_uint32 length;
    comma::packed::net_float64 time_since_boot;
    comma::packed::string< 240 > dummy1;
    boost::array< comma::packed::net_float64, number_of_joints > actual_joint_positions; 
    boost::array< comma::packed::net_float64, number_of_joints > actual_joint_velocities;
    boost::array< comma::packed::net_float64, number_of_joints > actual_joint_currents;
    comma::packed::string< 144 > dummy2;
    boost::array< comma::packed::net_float64, number_of_tool_fields > tool_generalised_forces;
    boost::array< comma::packed::net_float64, number_of_tool_fields > tool_pose;
    boost::array< comma::packed::net_float64, number_of_tool_fields > tool_speed;
    comma::packed::string< 8 > dummy3;
    boost::array< comma::packed::net_float64, number_of_joints >  joint_temperatures;
    comma::packed::string< 16 > dummy4;
    comma::packed::net_float64 robot_mode;
    boost::array< comma::packed::net_float64, number_of_joints > joint_modes;
};

} } //