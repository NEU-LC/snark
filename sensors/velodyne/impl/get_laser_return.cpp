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
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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
#include <snark/sensors/velodyne/impl/angle.h>
#include <snark/sensors/velodyne/impl/get_laser_return.h>

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
// which issued three upper block returns for every lower block return, the HDL-64E S2 has an
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
    return boost::posix_time::microseconds( offset * 1000000 );
}

double azimuth( double rotation, unsigned int laser, double angularSpeed )
{
    double a = rotation + angularSpeed * timestamps::step * laser + 90; // add 90 degrees for our system of coordinates (although this value is only output for later processing - can keep its own)
    if( comma::math::less( a, 360 ) ) { if( comma::math::less( a, 0 ) ) { a += 360; } }
    else { a -= 360; }
    return a;
}

double azimuth( const packet& packet, unsigned int block, unsigned int laser, double angularSpeed )
{
    return azimuth( double( packet.blocks[block].rotation() ) / 100, laser, angularSpeed );
}

static bool is_upper( unsigned int block ) { return ( block & 0x1 ) == 0; }

laser_return get_laser_return( const packet& packet
                             , unsigned int block
                             , unsigned int laser
                             , const boost::posix_time::ptime& timestamp
                             , double angularSpeed
                             , bool raw )
{
    laser_return r;
    r.id = laser + ( is_upper( block ) ? 0 : 32 );
    r.intensity = packet.blocks[block].lasers[laser].intensity();
    r.range = double( packet.blocks[block].lasers[laser].range() ) / 500;
    if( raw )
    {
        r.timestamp = timestamp;
        r.azimuth = double( packet.blocks[block].rotation() ) / 100;
    }
    else
    {
        r.timestamp = timestamp + time_offset( block, laser );
        r.azimuth = azimuth( packet, block, laser, angularSpeed );
    }
    return r;
}

} } } // namespace snark {  namespace velodyne { namespace impl {
