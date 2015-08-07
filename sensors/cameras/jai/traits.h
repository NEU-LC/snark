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

/// @author vsevolod vlaskine

#ifndef SNARK_SENSORS_JAI_TRAITS_H_
#define SNARK_SENSORS_JAI_TRAITS_H_

#include <string>
#include <comma/visiting/traits.h>
#include "camera.h"
#include "node.h"

namespace comma { namespace visiting {

template <> struct traits< snark::jai::camera::info >
{
    template < typename K, typename V > static void visit( const K&, snark::jai::camera::info& t, V& v )
    {
        v.apply( "manufacturer", t.manufacturer );
        v.apply( "model_name", t.model_name );
        v.apply( "ip_address", t.ip_address );
        v.apply( "mac_address", t.mac_address );
        v.apply( "serial_number", t.serial_number );
        v.apply( "username", t.username );
        //v.apply( "interface_id", t.interface_id );
    }
    
    template < typename K, typename V > static void visit( const K&, const snark::jai::camera::info& t, V& v )
    {
        v.apply( "manufacturer", t.manufacturer );
        v.apply( "model_name", t.model_name );
        v.apply( "ip_address", t.ip_address );
        v.apply( "mac_address", t.mac_address );
        v.apply( "serial_number", t.serial_number );
        v.apply( "username", t.username );
        //v.apply( "interface_id", t.interface_id );
    }
};

template <> struct traits< snark::jai::node >
{
    template < typename K, typename V > static void visit( const K&, const snark::jai::node& t, V& v )
    {
        v.apply( "name", t.name );
        v.apply( "value", t.value );
        v.apply( "type", std::string( t.type_as_string() ) );
        v.apply( "access", std::string( t.access_as_string() ) );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_SENSORS_JAI_TRAITS_H_
