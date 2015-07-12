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

/// @author vsevolod vlaskine

#ifndef SNARK_NAVIGATION_NMEA_MESSAGES_H_
#define SNARK_NAVIGATION_NMEA_MESSAGES_H_

#include <boost/date_time/posix_time/ptime.hpp>
#include <comma/string/string.h>
#include "../../math/spherical_geometry/coordinates.h"
#include "message.h"
#include "string.h"

namespace snark { namespace nmea { namespace messages {

struct coordinate { double value; };

struct coordinates
{
    coordinate latitude;
    coordinate longitude;
    
    spherical::coordinates operator()() const { return spherical::coordinates( latitude.value, longitude.value ); }
};

struct time { boost::posix_time::ptime value; };

struct angle { double value; };
    
struct gpgga
{
    static const char* name() { return "GPGGA"; }
    
    struct quality_t { enum values { fix_not_valid = 0, gps_fix = 1, differential_gps_fix = 2, real_time_kinematic_fixed_integers = 4, real_time_kinematic_float_integers = 5 }; };
    
    nmea::messages::time time;
    messages::coordinates coordinates;
    quality_t::values quality;
    unsigned int satellites_in_use;
    double hdop;
    double orthometric_height;
    std::string height_unit;
    double geoid_separation;
    std::string geoid_separation_unit;
    double age_of_differential_gps_data_record;
    std::string reference_station_id;
};

namespace ptnl
{
    struct string : public nmea::string { const std::string& ptnl_type() const { return values()[1]; } };
    
    template < typename T > struct value : public nmea::message< T >
    { 
        static const char* name() { return "PTNL"; }
        typedef nmea::message< T > type;
    };
    
    struct avr
    {
        static const char* name() { return "AVR"; }
        
        struct quality_t { enum values { fix_not_valid = 0, autonomous_gps_fix = 1, differential_carrier_phase_solution_rtk_float = 2, differential_carrier_phase_solution_rtk_int = 3, differential_code_based_solution_dgps = 4 }; };
        
        nmea::messages::time time;
        nmea::messages::angle yaw;
        std::string yaw_string;
        nmea::messages::angle tilt;
        std::string tilt_string;
        nmea::messages::angle roll;
        std::string roll_string;
        double range;
        quality_t::values quality;
        double pdop;
        unsigned int satellites_in_use;
    };
};
    
} } } // namespace snark { namespace nmea { namespace messages {

#endif // SNARK_NAVIGATION_NMEA_MESSAGES_H_
