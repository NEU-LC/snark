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

#include <gtest/gtest.h>
#include <snark/sensors/sick/cola/binary/packets.h>

using namespace snark::sick::cola::binary;

TEST( cola, binary_packet_basics )
{
    packet< payloads::set_access_mode::request > request;
    // todo: test something...
}

TEST( cola, scan_packet )
{
    // example packet from Section 6.4.1 p40 of Sick LMS151 telegram documentation v1.0 (04.07.14 - 8016687/0000/2014-07-04)
    // Note this doesn't exercise much of the packet at all (most optional components are not present, only 1x 16-bit channel)
    const char buf[] = { 0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x83,0x73,0x52,0x41,0x20,0x4C,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74,0x61,0x20,0x00,0x01,0x00,0x01,
                            0x00,0x89,0xA2,0x7F,0x00,0x00,0xC8,0xC8,0xC8,0xCC,0x15,0x58,0x86,0xD8,0x15,0x58,0x8C,0x5A,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x13,0x88,
                            0x00,0x00,0x01,0x68,0x00,0x00,0x00,0x01,0x44,0x49,0x53,0x54,0x31,0x3F,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x86,0xA0,0x13,0x88,0x00,
                            0x15,0x08,0x93,0x08,0x95,0x08,0xAF,0x08,0xB3,0x08,0xB0,0x08,0xA4,0x08,0xB0,0x08,0xBF,0x08,0xB9,0x08,0xBA,0x08,0xD0,0x08,0xD3,0x08,0xCF,0x08,
                            0xDE,0x08,0xEB,0x08,0xE3,0x08,0xFE,0x08,0xEC,0x09,0x03,0x08,0xFD,0x08,0xFD,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2B };

    scan_packet p = scan_packet(buf); //reinterpret_cast< scan_packet* >( buf );
    std::cerr << "valid     : " << std::boolalpha << p.valid() << std::endl;
    std::cerr << "type      : " << p.body_header().command_type() << ":" << p.body_header().type() << std::endl;
    std::cerr << "length    : " << p.header().length() << " B" << std::endl;
    std::cerr << "serial no : " << "0x" << std::hex << p.device().serial_number() << std::dec << std::endl;
    std::cerr << "uptime    : " << p.status().time_since_boot() << " us" << std::endl;
    std::cerr << "digitalout: " << "0x" << std::hex << p.status().digital_output_state() << std::dec << std::endl;
    std::cerr << "scan freq : " << p.frequency().scan_frequency() / 100.0 << " Hz" << std::endl;
    std::cerr << "shot freq : " << p.frequency().measurement_frequency() / 100.0 << " Hz" << std::endl; // todo: not sure if scaling is really 100 Hz
    std::cerr << "encoders  : " << p.encoders().encoders_size() << std::endl;
    std::cerr << "chans 16b : " << p.channels16().channels_size() << std::endl;
    const scan_packet::channel16_t* c16 = NULL;
    for ( unsigned int ch16id = 0; ch16id < p.channels16().channels_size(); ++ch16id )
    {
        c16 = p.channels16().channels_next(c16);
        std::cerr << "chan16[" << ch16id << "] : " << c16->channel_content()
                  << ", x" << c16->scale_factor()
                  << ", +" << c16->scale_offset()
                  << ", start_angle=" << c16->start_angle() / 10000.0 << " deg"
                  << ", steps=" << c16->steps() / 10000.0 << " deg"
                  << ", length=" << c16->data_size() << std::endl;
    }
    std::cerr << "chans 8b  : " << p.channels8().channels_size() << std::endl;
    const scan_packet::channel8_t* c8 = NULL;
    for ( unsigned int ch8id = 0; ch8id < p.channels8().channels_size(); ++ch8id )
    {
        c8 = p.channels8().channels_next(c8);
        std::cerr << "chan8[" << ch8id << "]  : " << c8->channel_content()
                  << ", x" << c8->scale_factor()
                  << ", +" << c8->scale_offset()
                  << ", start_angle=" << c8->start_angle() / 10000.0 << " deg"
                  << ", steps=" << c8->steps() / 10000.0 << " deg"
                  << ", length=" << c8->data_size() << std::endl;
    }
    std::cerr << "position  : " << p.position().data_present() << std::endl;
    std::cerr << "name      : " << p.name().data_present() << std::endl;
    std::cerr << "comment   : " << p.comment().data_present() << std::endl;
    std::cerr << "timestamp : " << p.time().data_present() << std::endl;
    std::cerr << "event     : " << p.event().data_present() << std::endl;
    std::cerr << "crc       : " << "0x" << std::hex << (unsigned int)p.crc() << std::dec << std::endl;

    // todo: find a packet that includes the timestamp
    // std::cerr << "seconds   : " << p->t.seconds() << std::endl;
    // std::cerr << "fractions : " << p->t.fractions() << std::endl;

    // boost::posix_time::ptime e( snark::timing::epoch );
    // boost::posix_time::ptime t( e );
    // t -= ( boost::posix_time::seconds( 1 ) + boost::posix_time::microsec( 1000 ) );
    // std::cerr << "---> total seconds: " << ( t - e ).total_seconds() << std::endl;
    // std::cerr << "---> microseconds: " << ( ( t - e ).total_microseconds() - ( t - e ).total_seconds() * 1000000 ) << std::endl;
}
