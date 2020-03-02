// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2020 Mission Systems Pty Ltd
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright owner nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
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

#include <chrono>
#include <vector>
#include <comma/string/split.h>
#include "echodyne.h"
#include "types.h"
#include "radar.h"

namespace snark { namespace echodyne {

radar::radar()
{
    radar_ = std::make_unique< bnet_interface >();
}

radar::~radar()
{
    if( radar_ )
    {
        comma::verbose << "disconnecting from radar" << std::endl;
        radar_->disconnect();
    }
}

void radar::connect( const std::string& address, int port )
{
    comma::verbose << "connecting to " << address << ":" << port << std::endl;
    radar_->connect( address.c_str(), port, "" );
}

void radar::command( const std::string& cmd )
{
    std::vector< std::string > cmd_words = comma::split( cmd );
    if( cmd.empty() ) { return; }
    else if( cmd == "API:BUFFERS" ) { show_buffer_states(); }
    else if( cmd == "API:SYS_STATE" ) { show_system_state(); }
    else if( cmd_words[0] == "API:ENABLE_BUFFER" )
    {
        try { enable_buffer( snark::echodyne::mesa_data_from_string( cmd_words[1] )); }
        catch( std::invalid_argument& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    }
    else if( cmd_words[0] == "API:DISABLE_BUFFER" )
    {
        try { disable_buffer( snark::echodyne::mesa_data_from_string( cmd_words[1] )); }
        catch( std::invalid_argument& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    }
    else { send_command( cmd, true ); }
}

std::pair< unsigned int, unsigned int > radar::get_time()
{
    auto [ success, response ] = send_command( "SYS:TIME?", comma::verbose );
    if( !success ) { throw std::runtime_error( "" ); }
    std::vector< std::string > lines = comma::split( response, '\n' );
    if( lines.size() == 0 ) { throw std::runtime_error( "tried to get time offset, got no response" ); }
    std::vector< std::string > words = comma::split( lines[0], ',' );
    if( words.size() != 2 ) { throw std::runtime_error( "tried to get time offset, got \"" + lines[0] + "\"" ); }
    unsigned int days;
    unsigned int milliseconds;
    try
    {
        days = std::stoul( words[0] );
        milliseconds = std::stoul( words[1] );
    }
    catch( ... )
    {
        throw std::runtime_error( "tried to read time offset, got \"" + lines[0] + "\"" );
    }
    return std::pair( days, milliseconds );
}

void radar::set_time_offset( unsigned int days, unsigned int milliseconds )
{
    bool success;
    std::string response;
    std::tie( success, response ) = send_command( "SYS:TIME " + std::to_string( days ) + "," + std::to_string( milliseconds ), comma::verbose );
    if( !success ) { throw std::runtime_error( "" ); }
}

// see section 8.9 of the User Manual for the algorithm
// essentially we are setting the offset to be from unix epoch rather than radar boot time
void radar::set_time()
{
    using namespace std::chrono;
    comma::verbose << "setting time" << std::endl;

    // clear any existing offsets and get the time from boot
    set_time_offset( 0, 0 );
    auto [ days, ms ] = get_time();
    system_clock::duration current_offset( seconds( days * 86400 ));
    current_offset += milliseconds( ms );

    // get the current system time, calculate the offset required for that time, and set that offset
    system_clock::time_point now = system_clock::now();
    system_clock::time_point required_offset = now - current_offset;
    auto required_offset_days( time_point_cast< seconds >( required_offset ).time_since_epoch().count() / 86400 );
    auto required_offset_ms( time_point_cast< milliseconds >( required_offset ).time_since_epoch().count() - required_offset_days * 86400000 );
    comma::verbose << "sending offset to set time to " << boost::posix_time::to_iso_string( boost::posix_time::from_time_t( system_clock::to_time_t( now ))) << " (plus some milliseconds)" << std::endl;
    set_time_offset( required_offset_days, required_offset_ms );

    // check that the radar now reports the correct time in unix epoch
    std::tie( days, ms ) = get_time();
    now = system_clock::now();
    comma::verbose << "time set to days = " << days << "; milliseconds = " << ms << std::endl;
    system_clock::time_point radar_time( seconds( days * 86400 ));
    radar_time += milliseconds( ms );
    auto error_ms = duration_cast< milliseconds >( radar_time - now ).count();
    if( std::abs( error_ms ) < 10 ) { comma::verbose << "error with respect to system time = " << error_ms << "ms" << std::endl; }
    else { std::cerr << comma::verbose.app_name() << ": *WARNING* error with respect to system time = " << error_ms << "ms" << std::endl; }
}

std::pair< bool, std::string > radar::send_command( const std::string& cmd, bool verbose )
{
    comma::verbose << "sending " << cmd << std::endl;
    auto [ status, response ] = radar_->send_command( cmd );
    if( status != MESA_OK )
    {
        auto response_lines = comma::split( response, '\n' );
        for( auto& s : response_lines ) { s.insert( 0, "        " ); }
        std::cerr << comma::verbose.app_name() << ": " << "send_command: \"" << cmd << "\""
                  << "\n    failed with status " << mesa_command_status_to_string( status )
                  << ", response was:\n" << comma::join( response_lines, '\n' ) << std::endl;
    }
    if( verbose ) { std::cerr << response << std::endl; }
    return std::pair( status == MESA_OK, response );
}

void radar::show_buffer_state( mesa_data_t d_type )
{
    std::cerr << radar_->get_n_buffered( d_type ) << " / " << radar_->get_buffer_length( d_type );
    if( radar_->get_collect( d_type )) { std::cerr << " collecting"; }
    if( radar_->get_save( d_type )) { std::cerr << " saving"; }
}

void radar::show_buffer_states()
{
    std::cerr << "buffers:";
    std::cerr << "\n     status: "; show_buffer_state( STATUS_DATA );
    std::cerr << "\n      rvmap: "; show_buffer_state( RVMAP_DATA );
    std::cerr << "\n  detection: "; show_buffer_state( DETECTION_DATA );
    std::cerr << "\n      track: "; show_buffer_state( TRACK_DATA );
    std::cerr << "\n       meas: "; show_buffer_state( MEAS_DATA );
    std::cerr << std::endl;
}

void radar::show_system_state()
{
    MESAK_Status data_status_packet = radar_->get_status();
    std::cerr << "system state queried at " << data_status_packet.data->sys_time_ms << "ms" << std::endl;
    std::cerr  << "  system state = " << snark::echodyne::system_state_to_string( data_status_packet.data->sys_state )
               << " (" << data_status_packet.data->sys_state << ")" << std::endl;
}

} } // namespace snark { namespace echodyne {
