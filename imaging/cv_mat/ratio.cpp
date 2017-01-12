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

#include "ratio.h"

namespace snark{ namespace cv_mat {

namespace ratio
{

    std::ostream & operator<<( std::ostream & o, const channel & c )
    {
        switch( c.c )
        {
            case( channel::red ):      o << "r"; break;
            case( channel::green ):    o << "g"; break;
            case( channel::blue ):     o << "b"; break;
            case( channel::alpha ):    o << "a"; break;
            default:                             break;
        }
        return o;
    }

    std::ostream & operator<<( std::ostream & o, const term & t )
    {
        switch( t.c )
        {
            case( channel::NUM_CHANNELS ):
                break;
            case( channel::constant ):
                o << t.value; break;
            default:
                if ( t.value == 1.0 ) {
                    o << t.c;
                } else if ( t.value == -1.0 ) {
                    o << "-" << t.c;
                } else {
                  o << t.value << t.c;
                }
                break;
        }
        return o;
    }

    combination::combination()
    {
        terms.reserve( channel::NUM_CHANNELS );
        for ( int ci = channel::constant; ci != channel::NUM_CHANNELS ; ++ci )
        {
            channel::channel_types c = static_cast< channel::channel_types >( ci );
            terms.push_back( term( 0.0, c ) );
        }
    }

    void combination::update( const term & t )
    {
        std::vector< term >::iterator i = terms.begin() + ( t.c - channel::constant );
        i->value += t.value;
        if ( std::abs( i->value ) < epsilon ) { i->value = 0.0; }
    }

    std::string combination::stringify( ) const
    {
        std::ostringstream o;
        std::vector< term >::const_iterator i = terms.begin();
        // constant term is always the first; optionally output zero
        while ( i != terms.end() )
        {
            bool separate = false;
            if ( i->value != 0.0 || terms.size() == 1 ) { o << *i; separate = true; }
            ++i;
            if ( separate && i != terms.end() ) { o << " "; }
        }
        return o.str();
    }

    void combination::print( std::ostream & o ) const
    {
        for ( std::vector< term >::const_iterator i = terms.begin(); i != terms.end(); )  // incremented explicitly
        {
            o << i->value; o << ( ++i == terms.end() ? "" : "," );
        }
    }

    const double combination::epsilon = 1.0e-10;

}

} }  // namespace snark { namespace cv_mat {
