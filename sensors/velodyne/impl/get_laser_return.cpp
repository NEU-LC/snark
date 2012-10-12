// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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

laser_return getlaser_return( const packet& packet
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
