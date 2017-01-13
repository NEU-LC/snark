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

#include <comma/application/verbose.h>

namespace snark{ namespace cv_mat {

namespace ratios
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

    size_t combination::non_zero_term_count() const
    {
        // no c++-11
        struct counter
        {
            static size_t non_zero( size_t count, const term& t )
            {
                return count + ( t.value != 0.0 );
            }
        };
        return std::accumulate( terms.begin(), terms.end(), 0, counter::non_zero );
    }

    std::string combination::stringify( ) const
    {
        std::ostringstream o;
        std::vector< term >::const_iterator i = terms.begin();
        size_t ncount = non_zero_term_count();
        bool previous = false;
        for ( std::vector< term >::const_iterator i = terms.begin(); i != terms.end(); ++i )
        {
            // constant term is always the first; optionally output zero
            if ( i->value != 0.0 || ( i == terms.begin() && ncount == 0 ) )
            {
                if ( previous ) { o << ( i->value > 0.0 ? " + " : " - " ) << abs(*i); } else { o << *i; }
                previous = true;
            }
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

    void usage( std::ostream & o, bool verbose )
    {
        o << comma::verbose.app_name() << ": this help describes input format for ratio operation\n";
        o << "\n";
        o << "    input string specifies a ratio of linear combinations of signal channels in algebraic form\n";
        o << "    one of the following formats can be used:\n";
        o << "            ( linear combination ) / ( linear combination )\n";
        o << "        here 'linear combination' is in the form Ar + Bg + Cb + Da + F, where A, B, C, D, F\n";
        o << "        are floating-point constants and r, g, b, a are channel names; a multiplication sign '*'\n";
        o << "        is allowed between the constant and channel name; the F term gives a constant offset\n";
        o << "        to be added to the output\n";
        o << "            term / ( linear combination )\n";
        o << "        here 'term' is either A or Ac, where A is a constant and c is one of channel names\n";
        o << "        for a single term, surrounding brackets are optional, 'term' is the same as '(term)'\n";
        o << "            ( linear combination ) / term\n";
        o << "            term / term\n";
        o << "            linear combination\n";
        o << "        here the linear combination becomes the ratio numerator, while the denominator is set to 1\n";
        o << "        brackets are optional around the numerator-only linear combination\n";
        o << "\n";
        o << "    the input is parsed into two lists of coefficients for the numerator and denominator\n";
        o << "\n";
        o << "examples (outputs are shown after --> arrows as ratios of comma-separated lists of coefficients,\n";
        o << "for the numerator and denominator)\n";
        o << "    ratios are composed of linear combinations in the natural way\n";
        o << "        ( a +2*b - 3r ) / ( r + b )  -->  0,-3,0,2,1 / 0,1,0,1,0\n";
        o << "        a / ( r + b )                -->  0,0,0,0,1 / 0,1,0,1,0\n";
        o << "        (3a + 4*b) / g               -->  0,0,0,4,3 / 0,0,1,0,0\n";
        o << "        3a / g                       -->  0,0,0,0,3 / 0,0,1,0,0\n";
        o << "    terms and signs between them may be separated by spaces\n";
        o << "    brackets are optional for single terms and are mandatory for multi-term linear combinations\n";
        o << "    with one exception, numerator-only input\n";
        o << "        3a + b / g                   -->  error!\n";
        o << "        3a + b                       -->  0,0,0,1,3 / 1,0,0,0,0\n";
        o << "    multiplication signs are optional and multipliers of 1 or -1 can be omitted\n";
        o << "        ( 2*b - a ) / 2              -->  0,0,0,2,-1 / 2,0,0,0,0\n";
        o << "    leading '+' sign is ignored; arbitrary floating-point notation is supported\n";
        o << "        +2*b - 1.3e-4r               -->  0,-1.3e-4,0,2,0 / 1,0,0,0,0\n";
        o << "    order of channel terms does not matter\n";
        o << "        1 + r - g + b                -->  1,1,-1,1,0 / 1,0,0,0,0\n";
        o << "        b - g + 1 + r                -->  1,1,-1,1,0 / 1,0,0,0,0\n";
        o << std::endl;
    }

} // namespace ratios

} }  // namespace snark { namespace cv_mat {
