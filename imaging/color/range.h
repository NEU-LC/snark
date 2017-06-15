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

#pragma once

#include <comma/base/types.h>
#include <string>
#include <type_traits>

namespace snark { namespace imaging {

    enum range {
        ub = 0,   // 0 - 255
        uw,       // 0 - 65535
        ui,       // 0 - 4294967295
        f,        // 0 - 1
        d         // 0 - 1
    };

    struct stringify
    {
        static std::string from( range r );
        static range to( const std::string & s );
    };

    template< range R > struct range_traits;
    template<> struct range_traits< ub > { typedef unsigned char value_t; };
    template<> struct range_traits< uw > { typedef comma::uint16 value_t; };
    template<> struct range_traits< ui > { typedef comma::uint32 value_t; };
    template<> struct range_traits< f >  { typedef float         value_t; };
    template<> struct range_traits< d >  { typedef double        value_t; };

    template< range R >
    struct limits
    {
        typedef typename range_traits< R >::value_t value_t;
        static constexpr double upper() { return std::is_floating_point< value_t >::value ? 1.0 : double( std::numeric_limits< value_t >::max() ); }
        static constexpr double lower() { return 0.0; }
    };

    template< typename T, range R >
    T trim( T t ) {
        double dt = double(t);
        return dt < limits< R >::lower() ? static_cast< T >( limits< R >::lower() ) : ( dt > limits< R >::upper() ? static_cast< T >( limits< R >::upper() ) : t );
    }

} } // namespace snark { namespace imaging {
