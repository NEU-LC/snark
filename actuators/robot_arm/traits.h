#ifndef SNARK_ACTUATORS_ROBOT_ARM_TRAITS_H
#define SNARK_ACTUATORS_ROBOT_ARM_TRAITS_H

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


#include "commands.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {
    
namespace robot_arm = snark::robot_arm;
using snark::robot_arm::command_base;

// Commands
template <typename C> struct traits< command_base< C > >
{
    template< typename K, typename V > static void visit( const K& k, command_base< C >& t, V& v )
    {
        v.apply( "rover_id", t.rover_id );
        v.apply( "sequence_number", t.sequence_number );
        v.apply( "name", t.name );
    }
    template< typename K, typename V > static void visit( const K& k, const command_base< C >& t, V& v )
    {
        v.apply( "rover_id", t.rover_id );
        v.apply( "sequence_number", t.sequence_number );
        v.apply( "name", t.name );
    }
};

template <> struct traits< robot_arm::move_cam >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::move_cam& t, V& v )
    {
    	traits< command_base < robot_arm::move_cam > >::visit(k, t, v);
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
        v.apply( "height", t.height );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::move_cam& t, V& v )
    {
    	traits< command_base < robot_arm::move_cam > >::visit(k, t, v);
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
        v.apply( "height", t.height );
    }
};

template <> struct traits< robot_arm::position >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::position& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::position& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }
};

template <> struct traits< robot_arm::move_effector >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::move_effector& t, V& v )
    {
        traits< command_base < robot_arm::move_effector > >::visit(k, t, v);
        v.apply( "offset", t.offset );
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
        v.apply( "roll", t.roll );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::move_effector& t, V& v )
    {
        traits< command_base < robot_arm::move_effector > >::visit(k, t, v);
        v.apply( "offset", t.offset );
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
        v.apply( "roll", t.roll );
    }
};

template <> struct traits< robot_arm::move_joints >
{
    template< typename K, typename V > static void visit( const K& k, robot_arm::move_joints& t, V& v )
    {
        traits< command_base < robot_arm::move_joints > >::visit(k, t, v);
        v.apply( "joints", t.joints );
    }

    template< typename K, typename V > static void visit( const K& k, const robot_arm::move_joints& t, V& v )
    {
        traits< command_base < robot_arm::move_joints > >::visit(k, t, v);
        v.apply( "joints", t.joints );
    }
};


}} // namespace comma { namespace visiting {

#endif // SNARK_ACTUATORS_ROBOT_ARM_TRAITS_H
