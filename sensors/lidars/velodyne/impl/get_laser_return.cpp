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


#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/math/compare.h>
#include <comma/base/exception.h>
#include "angle.h"
#include "get_laser_return.h"

namespace snark {  namespace velodyne { namespace impl {

struct timestamps
{
    // time offsets of the laser blocks
    static const double offsets[];

    // time offset of the laser return written to packet first (see velodyne HDL-64E S2 Manual, Appendix D)
    static const double first;

    // time offset of the laser return written to packet last (see velodyne HDL-64E S2 Manual, Appendix D)
    static const double last;

    // time elapsed between first and last laser returns
    static const double elapsed;

    // number of steps between first and last laser returns
    // ( upper and lower blocks are fired at the same time, see velodyne HDL-64E S2 Manual, Appendix D)
    static const std::size_t size;

    // step size between two laser returns (a convenience definition)
    static const double step;

    // step size between two laser returns
    static const double ethernetOutputDuration;

    // return laser return offset in a laser block
    static double offsetInBlock( std::size_t laser ) { return step * laser; }
};

// see velodyne HDL-64ES2 User's Manual, p.19:
// First transmission trigger to Ethernet tranmission enable: 419.3 microseconds
const double timestamps::offsets[] = {   -0.0004193, -0.0004193
                                       , -0.0003960, -0.0003960
                                       , -0.0003727, -0.0003727
                                       , -0.0003494, -0.0003494
                                       , -0.0003261, -0.0003261
                                       , -0.0003028, -0.0003028 };

const double timestamps::first( timestamps::offsets[0] );
const double timestamps::last( -0.0002803 );
const double timestamps::elapsed( last - first );
const std::size_t timestamps::size( 6 * 32 );
const double timestamps::step( elapsed / ( timestamps::size - 1 ) ); // see velodyne HDL-64ES2 User's Manual, p.19
const double timestamps::ethernetOutputDuration( 0.0001 ); // Ethernet output duration: 100 microseconds

// In the HDL-64E S2, the upper block and lower block collect distance points simultaneously, with
// each block issuing single laser pulses at a time. That is, each upper block laser fires in
// sequence and in unison to a corresponding laser from the lower block. For example, laser 32
// fires simultaneously with laser 0, laser 33 fires with laser 1, and so on. Unlike the HDL-64E,
// which issued three upper block returns for every lowestepr block return, the HDL-64E S2 has an
// equal number of upper and lower block returns. This is why when interpreting the delay table
// each sequential pair of data blocks will represent the upper and lower block respectively,
// and each upper and lower block pair of data blocks in the Ethernet packet will have the same
// delay values.
//
// Ethernet packets are assembled until the entire 1200 bytes have been collected, representing
// six upper block sequences and six lower block sequences. The packet is then transmitted via
// a UDP packet over Ethernet, starting from the last byte acquired. See a sample of the packet
// layout on page 20.
//
// Simply subtract from the timestamp of the output event of the packet each data value
// to arrive at the actual time the distance point was captured inside the HDL-64E S2
boost::posix_time::time_duration time_offset( unsigned int block, unsigned int laser )
{
    double offset = timestamps::offsets[ block ] + timestamps::step * laser - timestamps::ethernetOutputDuration;
    return boost::posix_time::microseconds( static_cast< long >( offset * 1000000 ) );
}

double azimuth( double rotation, unsigned int laser, double angularSpeed )
{
    double a = rotation + angularSpeed * timestamps::step * laser + 90; // add 90 degrees for our system of coordinates (although this value is only output for later processing - can keep its own)
    if( comma::math::less( a, 360 ) ) { if( comma::math::less( a, 0 ) ) { a += 360; } }
    else { a -= 360; }
    return a;
}

double azimuth( const hdl64::packet& packet, unsigned int block, unsigned int laser, double angularSpeed )
{
    return azimuth( double( packet.blocks[block].rotation() ) / 100, laser, angularSpeed );
}

const int lasers_per_block=32;
const int block_count=12;
struct hdl64_s2_fw_v48
{
    typedef double block_time_table[lasers_per_block];
    static block_time_table time_table[block_count];
    static boost::posix_time::time_duration time_delay( unsigned int block, unsigned int laser )
    {
        if(laser<0 ||laser>=lasers_per_block) { COMMA_THROW( comma::exception, "laser id out of range" << laser ); }
        if(block<0||block>=block_count) { COMMA_THROW(comma::exception, "block id out of range"<<block ); }
        double delay = (time_table[block][laser] ) + timestamps::ethernetOutputDuration * 1e6;
        return boost::posix_time::microseconds( delay);
    }
    static double azimuth(const hdl64::packet& packet, unsigned int block, unsigned int laser, double angularSpeed )
    {
        // todo: angular speed correction with offset for angular velocity that calibration was measured on
        double rotation = double( packet.blocks[block].rotation() ) / 100;
        double laser_time = (time_table[block][0] - time_table[block][laser%lasers_per_block]) * 1e-6;
        double a = rotation + 90;
        a+=angularSpeed * laser_time;
        if( comma::math::less( a, 360 ) ) { if( comma::math::less( a, 0 ) ) { a += 360; } }
        else { a -= 360; }
        return a;
    }
};

//delay in microseconds for each data block, laser
//each upper lasers is fired with a lower laser at the same time, e.g. lasers 0 and 32 fire at the same time then 1 and 33 ...
//each row is for one block and each column is for one laser id 
hdl64_s2_fw_v48::block_time_table hdl64_s2_fw_v48::time_table[block_count]=
{
    { 288, 286.74, 285.54, 284.34, 282, 280.74, 279.54, 278.34, 276, 274.74, 273.54, 272.34, 270, 268.74, 267.54, 266.34, 264, 262.74, 261.54, 260.34, 258, 256.74, 255.54, 254.34, 252, 250.74, 249.54, 248.34, 246, 244.74, 243.54, 242.34 },
    { 288, 286.74, 285.54, 284.34, 282, 280.74, 279.54, 278.34, 276, 274.74, 273.54, 272.34, 270, 268.74, 267.54, 266.34, 264, 262.74, 261.54, 260.34, 258, 256.74, 255.54, 254.34, 252, 250.74, 249.54, 248.34, 246, 244.74, 243.54, 242.34 }, 
    { 240, 238.74, 237.54, 236.34, 234, 232.74, 231.54, 230.34, 228, 226.74, 225.54, 224.34, 222, 220.74, 219.54, 218.34, 216, 214.74, 213.54, 212.34, 210, 208.74, 207.54, 206.34, 204, 202.74, 201.54, 200.34, 198, 196.74, 195.54, 194.34 }, 
    { 240, 238.74, 237.54, 236.34, 234, 232.74, 231.54, 230.34, 228, 226.74, 225.54, 224.34, 222, 220.74, 219.54, 218.34, 216, 214.74, 213.54, 212.34, 210, 208.74, 207.54, 206.34, 204, 202.74, 201.54, 200.34, 198, 196.74, 195.54, 194.34 }, 
    { 192, 190.74, 189.54, 188.34, 186, 184.74, 183.54, 182.34, 180, 178.74, 177.54, 176.34, 174, 172.74, 171.54, 170.34, 168, 166.74, 165.54, 164.34, 162, 160.74, 159.54, 158.34, 156, 154.74, 153.54, 152.34, 150, 148.74, 147.54, 146.34 }, 
    { 192, 190.74, 189.54, 188.34, 186, 184.74, 183.54, 182.34, 180, 178.74, 177.54, 176.34, 174, 172.74, 171.54, 170.34, 168, 166.74, 165.54, 164.34, 162, 160.74, 159.54, 158.34, 156, 154.74, 153.54, 152.34, 150, 148.74, 147.54, 146.34 }, 
    { 144, 142.74, 141.54, 140.34, 138, 136.74, 135.54, 134.34, 132, 130.74, 129.54, 128.34, 126, 124.74, 123.54, 122.34, 120, 118.74, 117.54, 116.34, 114, 112.74, 111.54, 110.34, 108, 106.74, 105.54, 104.34, 102, 100.74, 99.54, 98.34 }, 
    { 144, 142.74, 141.54, 140.34, 138, 136.74, 135.54, 134.34, 132, 130.74, 129.54, 128.34, 126, 124.74, 123.54, 122.34, 120, 118.74, 117.54, 116.34, 114, 112.74, 111.54, 110.34, 108, 106.74, 105.54, 104.34, 102, 100.74, 99.54, 98.34 }, 
    { 96, 94.74, 93.54, 92.34, 90, 88.74, 87.54, 86.34, 84, 82.74, 81.54, 80.34, 78, 76.74, 75.54, 74.34, 72, 70.74, 69.54, 68.34, 66, 64.74, 63.54, 62.34, 60, 58.74, 57.54, 56.34, 54, 52.74, 51.54, 50.34 }, 
    { 96, 94.74, 93.54, 92.34, 90, 88.74, 87.54, 86.34, 84, 82.74, 81.54, 80.34, 78, 76.74, 75.54, 74.34, 72, 70.74, 69.54, 68.34, 66, 64.74, 63.54, 62.34, 60, 58.74, 57.54, 56.34, 54, 52.74, 51.54, 50.34 }, 
    { 48, 46.74, 45.54, 44.34, 42, 40.74, 39.54, 38.34, 36, 34.74, 33.54, 32.34, 30, 28.74, 27.54, 26.34, 24, 22.74, 21.54, 20.34, 18, 16.74, 15.54, 14.34, 12, 10.74, 9.54, 8.34, 6, 4.74, 3.54, 2.34 }, 
    { 48, 46.74, 45.54, 44.34, 42, 40.74, 39.54, 38.34, 36, 34.74, 33.54, 32.34, 30, 28.74, 27.54, 26.34, 24, 22.74, 21.54, 20.34, 18, 16.74, 15.54, 14.34, 12, 10.74, 9.54, 8.34, 6, 4.74, 3.54, 2.34 }, 
};

static bool is_upper( unsigned int block ) { return ( block & 0x1 ) == 0; }

laser_return get_laser_return( const hdl64::packet& packet
                             , unsigned int block
                             , unsigned int laser
                             , const boost::posix_time::ptime& timestamp
                             , double angularSpeed
                             , bool legacy)
{
    laser_return r;
    r.id = laser + ( is_upper( block ) ? 0 : 32 );
    r.intensity = packet.blocks[block].lasers[laser].intensity();
    r.range = double( packet.blocks[block].lasers[laser].range() ) / 500;
    if (legacy)
    {
        r.timestamp = timestamp + time_offset( block, laser );
        r.azimuth = azimuth( packet, block, laser, angularSpeed );
    }
    else
    {
        r.timestamp = timestamp - hdl64_s2_fw_v48::time_delay( block, laser );
        r.azimuth = hdl64_s2_fw_v48::azimuth(packet, block, laser, angularSpeed);
    }
    return r;
}

double time_span(bool legacy)
{
    if(legacy)
        return double( ( time_offset( 0, 0 ) - time_offset( 11, 0 ) ).total_microseconds() ) / 1e6;
    else
        return double( (hdl64_s2_fw_v48::time_delay(0,0) - hdl64_s2_fw_v48::time_delay(11,0) ).total_microseconds() ) / 1e6;
}

} } } // namespace snark {  namespace velodyne { namespace impl {
