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

#ifndef COMMA_UR_BASE
#define COMMA_UR_BASE

#include <comma/packed/packed.h>

namespace snark { namespace ur {

static const unsigned int number_of_pose_fields = 6;

template < typename T >
struct joint_values_t
{
    typedef T type;
    type base;
    type shoulder;
    type elbow;
    type wrist1;
    type wrist2;
    type wrist3;
};

struct arm_t
{
    typedef double type;
    joint_values_t< type > angles;
    joint_values_t< type > speeds;
    joint_values_t< type > currents;
    joint_values_t< type > temperatures;
    typedef int mode_t;
    joint_values_t< mode_t > modes;
};

template < typename T > struct coordinates_t 
{ 
    typedef T type; 
    type x;
    type y;
    type z;
};

struct pose_t
{
    typedef double type;
    coordinates_t< type > position;
    coordinates_t< type > orientation;
};

struct tool_t
{
    pose_t pose;
    pose_t velocity;
    pose_t force;
};

struct packed_joint_values_t : public comma::packed::packed_struct< packed_joint_values_t, 48 >, joint_values_t< comma::packed::net_float64 > 
{
    template < typename T >
    void export_to( joint_values_t< T >& v ) const
    {
        v.base = static_cast< T >( base() );
        v.shoulder = static_cast< T >( shoulder() );
        v.elbow = static_cast< T >( elbow() );
        v.wrist1 = static_cast< T >( wrist1() );
        v.wrist2 = static_cast< T >( wrist2() );
        v.wrist3 = static_cast< T >( wrist3() );
    }
};

struct packed_coordinates_t : public comma::packed::packed_struct< packed_coordinates_t, 24 >, coordinates_t< comma::packed::net_float64 >
{
    template < typename T >
    void export_to( coordinates_t< T >& c ) const { c.x = x(); c.y = y(); c.z = z(); }
};

struct packed_pose_t : public comma::packed::packed_struct< packed_pose_t, 48 >
{
    packed_coordinates_t position;
    packed_coordinates_t orientation;
    void export_to( pose_t& p ) const
    { 
        position.export_to< pose_t::type >( p.position ); 
        orientation.export_to< pose_t::type >( p.orientation ); 
    }
};

struct packet_t : public comma::packed::packed_struct< packet_t, 812  >
{
    comma::packed::net_uint32 length;
    comma::packed::net_float64 time_since_boot;
    comma::packed::string< 240 > dummy1;
    packed_joint_values_t actual_joint_angles;
    packed_joint_values_t actual_joint_speeds;
    packed_joint_values_t actual_joint_currents;
    comma::packed::string< 144 > dummy2;
    packed_pose_t tool_force;
    packed_pose_t tool_pose;
    packed_pose_t tool_velocity;
    comma::packed::string< 8 > dummy3;
    packed_joint_values_t joint_temperatures;
    comma::packed::string< 16 > dummy4;
    comma::packed::net_float64 mode;
    packed_joint_values_t joint_modes;
    
    void export_to( arm_t& arm ) const
    {
        actual_joint_angles.export_to< arm_t::type >( arm.angles );
        actual_joint_speeds.export_to< arm_t::type >( arm.speeds );
        actual_joint_currents.export_to< arm_t::type >( arm.currents );
        joint_temperatures.export_to< arm_t::type >( arm.temperatures );
        joint_modes.export_to< arm_t::mode_t >( arm.modes );
    }
    void export_to( tool_t& tool ) const
    {
        tool_pose.export_to( tool.pose );
        tool_velocity.export_to( tool.velocity ); 
        tool_force.export_to( tool.force );
    }
};

} } // namespace snark { namespace ur {

namespace comma { namespace visiting {    

template < typename T > struct traits< snark::ur::joint_values_t< T > >
{
    template< typename K, typename V > static void visit( const K&, snark::ur::joint_values_t< T >& t, V& v )
    {
        v.apply( "base", t.base );
        v.apply( "shoulder", t.shoulder );
        v.apply( "elbow", t.elbow );
        v.apply( "wrist1", t.wrist1 );
        v.apply( "wrist2", t.wrist2 );
        v.apply( "wrist3", t.wrist3 );
    }
    template< typename K, typename V > static void visit( const K&, const snark::ur::joint_values_t< T >& t, V& v )
    {
        v.apply( "base", t.base );
        v.apply( "shoulder", t.shoulder );
        v.apply( "elbow", t.elbow );
        v.apply( "wrist1", t.wrist1 );
        v.apply( "wrist2", t.wrist2 );
        v.apply( "wrist3", t.wrist3 );
    }
};

template <> struct traits< snark::ur::arm_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::arm_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "speeds", t.speeds );
        v.apply( "currents", t.currents );
        v.apply( "temperatures", t.temperatures );
        v.apply( "modes", t.modes );
    }    
    template< typename K, typename V > static void visit( const K& k, const snark::ur::arm_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "speeds", t.speeds );
        v.apply( "currents", t.currents );
        v.apply( "temperatures", t.temperatures );
        v.apply( "modes", t.modes );
    }
};

template < typename T > struct traits< snark::ur::coordinates_t< T > >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::coordinates_t< T >& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }    
    template< typename K, typename V > static void visit( const K& k, const snark::ur::coordinates_t< T >& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }
};

template <> struct traits< snark::ur::pose_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::pose_t& t, V& v )
    {
        v.apply( "position", t.position );
        v.apply( "orientation", t.orientation );
    }    
    template< typename K, typename V > static void visit( const K& k, const snark::ur::pose_t& t, V& v )
    {
        v.apply( "position", t.position );
        v.apply( "orientation", t.orientation );
    }
};

template <> struct traits< snark::ur::tool_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::tool_t& t, V& v )
    {
        v.apply( "pose", t.pose );
        v.apply( "velocity", t.velocity );
        v.apply( "force", t.force );
    }    
    template< typename K, typename V > static void visit( const K& k, const snark::ur::tool_t& t, V& v )
    {
        v.apply( "pose", t.pose );
        v.apply( "velocity", t.velocity );
        v.apply( "force", t.force );
    }
};

} } // namespace comma { namespace visiting {    

#endif // #ifndef COMMA_UR_BASE