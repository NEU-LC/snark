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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_TRAITS_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_TRAITS_H

#include <boost/units/quantity.hpp>
#include <comma/visiting/traits.h>
#include <snark/visiting/eigen.h>
#include "commands.h"
#include "commands_handler.h"

namespace comma { namespace visiting {
    
using snark::ur::command_base;

template <typename C> struct traits< command_base< C > >
{
    template< typename K, typename V > static void visit( const K& k, command_base< C >& t, V& v )
    {
        v.apply( "name", t.name );
    }
    template< typename K, typename V > static void visit( const K& k, const command_base< C >& t, V& v )
    {
        v.apply( "name", t.name );
    }
};

template <> struct traits< snark::ur::power >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::power& t, V& v ) {
        traits< command_base < snark::ur::power > >::visit(k, t, v);
        v.apply( "is_on", t.is_on );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::power& t, V& v ) {
        traits< command_base < snark::ur::power > >::visit(k, t, v);
        v.apply( "is_on", t.is_on );
    }
};

template <> struct traits< snark::ur::brakes >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::brakes& t, V& v ) {
        traits< command_base < snark::ur::brakes > >::visit(k, t, v);
        v.apply( "enable", t.enable );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::brakes& t, V& v ) {
        traits< command_base < snark::ur::brakes > >::visit(k, t, v);
        v.apply( "enable", t.enable );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_TRAITS_H
