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

#pragma once
#include "cepton.h"
#include <comma/csv/traits.h>
#include "../../../visiting/eigen.h"

namespace comma { namespace visiting {
   

template <> struct traits< snark::cepton::point_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::cepton::point_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "intensity", p.intensity );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::cepton::point_t& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "intensity", p.intensity );
    }
};

template <> struct traits< CeptonSensorInformation >
{
    template< typename K, typename V > static void visit( const K& k, const CeptonSensorInformation& p, V& v )
    {
        v.apply( "serial_number", p.serial_number );
        //or use std::vector<char>
        v.apply( "model_name", std::string(p.model_name) );
        v.apply( "model", p.model );
        v.apply( "firmware_version", std::string(p.firmware_version) );
        v.apply( "last_reported_temperature", p.last_reported_temperature );
        v.apply( "last_reported_humidity", p.last_reported_humidity );
        v.apply( "last_reported_age", p.last_reported_age );
        v.apply( "gps_ts_year", p.gps_ts_year );
        v.apply( "gps_ts_month", p.gps_ts_month );
        v.apply( "gps_ts_day", p.gps_ts_day );
        v.apply( "gps_ts_hour", p.gps_ts_hour );
        v.apply( "gps_ts_min", p.gps_ts_min );
        v.apply( "gps_ts_sec", p.gps_ts_sec );
        v.apply( "sensor_index", p.sensor_index );
        v.apply( "is_mocked", p.is_mocked );
        v.apply( "is_pps_connected", p.is_pps_connected );
        v.apply( "is_nmea_connected", p.is_nmea_connected );
        v.apply( "is_calibrated", p.is_calibrated );
    }
};
    
} } // namespace comma { namespace visiting {
    

