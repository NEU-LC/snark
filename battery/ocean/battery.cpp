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

#include "battery.h"
#include <boost/static_assert.hpp>

namespace snark { namespace ocean {
    
std::string battery_t::state_to_string( int st ) 
{
    switch( st )
    {
        case battery_state::initialised:
            return "IN";
            break;
        case battery_state::uninitialised:
            return "UN";
            break;
        case battery_state::fully_discharged:
            return "FD";
            break;
        case battery_state::fully_charged:
            return "FC";
            break;
        case battery_state::discharging:
            return "DC";
            break;
        default:
            return "CH";
            break;

    }
}
// Removes checksum wrappers, TODO throws exception on incorrect checksum
std::string& battery_t::strip( std::string& line )
{
    /// '$B15,....,FF00%B2' becomes B15,....,FF00
    //std::size_t pos = ( line[ line.size() - 3 ] == '%' ? line.size()-4 : std::string::npos );
    std::size_t pos = line.find_first_of( '%', line.size() - 4 );
    if( pos != std::string::npos ) { line = line.substr( 0, pos); }
    //std::cerr << "char: " << line[ line.size() - 3 ] << std::endl;
    ///line = line.substr( 0, pos);
    return line;
}

void battery_t::operator&(const data_t& data)
{
    // std::cerr << " address " << data.address() << std::endl;
    switch( data.address() )
    {
        case address::temperature:
        {
            static const double unit = 0.1; // Kelvin
            temperature = data.value() * unit * kelvin; // 0.1k unit
            // std::cerr << "got temperature: " << temperature.value() << std::endl;
            break;
        }
        case address::voltage:
        {
            voltage = data.value() / 1000.0 * volt; // millivolts to volts
            // std::cerr << "got voltage: " << voltage.value() << std::endl;
            break;
        }
        case address::current:
        {
            current = data.value.cast() / 1000.0 * ampere; //mAmp to Amps
            // std::cerr << "got current: " << current.value() << std::endl;
            break;
        }
        case address::average_current:
        {
            average_current = data.value.cast() / 1000.0 * ampere; //mAmp to Amps
            // std::cerr << "got average_current: " << average_current.value() << std::endl;
            break;
        }
        case address::remaining_capacity:
        {
            remaining_capacity = data.value.cast() / 100.0 * watt; // eacho unit is 10mWh - to Watts
        }
        case address::rel_state_of_charge:
        {
            charge_pc = data.value();    // percentage, unit is %
            break;
        }
        case address::run_time_to_empty:
        {
            time_to_empty = boost::posix_time::minutes( data.value() );
        }
        case address::status:
        {
            if( !(data.value() &  battery_state::initialised) ) 
            {
                state = battery_state::uninitialised;
                return;
            }
            comma::uint16 val = data.value() & 0x0070;  // masks out everything including 'initialised' flag
            switch( val )
            {
                case battery_state::discharging:
                    state = battery_state::discharging;
                    break;
                case battery_state::fully_charged:
                    state = battery_state::fully_charged;
                    break;
                case battery_state::fully_discharged:
                    state = battery_state::fully_discharged;
                    break;
                default:
                    state = battery_state::charging;
                    break;
            }
            // std::cerr << "battery: " << int(id) <<  " state: " << state << " value: " << data.value() << " val: " << val <<std::endl;
            break;
        }
        default:
        {
            return;
        }
    }
}
    
} } // namespace snark { namespace ocean {
