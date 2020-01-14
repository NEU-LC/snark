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

#include "ratio.h"

namespace snark{ namespace cv_mat {

namespace ratios
{
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;
    namespace phoenix = boost::phoenix;

    template< typename Iterator >
    rules< Iterator >::rules()
    {
        using qi::double_;
        using qi::_1;
        using qi::lit;
        using phoenix::bind;
        using qi::_val;
        using qi::eps;

        channel_ = eps[ _val = ratios::channel() ] >> ( lit('r')[ _val = channel::red ] | lit('g')[ _val = channel::green ] | lit('b')[ _val = channel::blue ] | lit('a')[ _val = channel::alpha ] );
        term_ = eps[ _val = ratios::term( 1.0, channel::constant ) ] >>
            (
                    double_[ bind( &term::value, _val ) = _1 ] >> -lit('*') >> channel_[ bind( &term::c, _val ) = _1 ]
                |   channel_[ bind( &term::c, _val ) = _1 ] >> lit('*') >> double_[ bind( &term::value, _val ) = _1 ]
                |   channel_[ bind( &term::c, _val ) = _1 ]
                |   lit('+')[ bind( &term::value, _val ) = 1 ] >> -( double_[ bind( &term::value, _val ) *= _1 ] >> -lit('*') ) >> channel_[ bind( &term::c, _val ) = _1 ]
                |   lit('-')[ bind( &term::value, _val ) = -1 ] >> -( double_[ bind( &term::value, _val ) *= _1 ] >> -lit('*') ) >> channel_[ bind( &term::c, _val ) = _1 ]
                |   lit('+')[ bind( &term::value, _val ) = 1 ] >> double_[ bind( &term::value, _val ) *= _1 ]
                |   lit('-')[ bind( &term::value, _val ) = -1 ] >> double_[ bind( &term::value, _val ) *= _1 ]
                |   double_[ bind( &term::value, _val ) = _1 ]
            );
        combination_ = eps[ _val = ratios::combination() ] >> +( term_[ bind( &combination::update, _val, _1 ) ] );
        ratio_ = eps[ _val = ratios::ratio() ] >>
            (
                    lit('(') >> combination_[ bind( &ratio::numerator, _val ) = _1 ] >> lit(')') >> lit('/') >> lit('(') >> combination_[ bind( &ratio::denominator, _val ) = _1 ] >> lit(')')
                |   lit('(') >> combination_[ bind( &ratio::numerator, _val ) = _1 ] >> lit(')') >> lit('/') >> term_[ bind( &ratio::denominator, _val ) = _1 ]
                |   term_[ bind( &ratio::numerator, _val ) = _1 ] >> lit('/') >> lit('(') >> combination_[ bind( &ratio::denominator, _val ) = _1 ] >> lit(')')
                |   term_[ bind( &ratio::numerator, _val ) = _1 ] >> lit('/') >> term_[ bind( &ratio::denominator, _val ) = _1 ]
                |   lit('(') >> combination_[ bind( &ratio::numerator, _val ) = _1 ] >> lit(')')
                |   combination_[ bind( &ratio::numerator, _val ) = _1 ]
            );
    }

} // namespace ratios

} }  // namespace snark { namespace cv_mat {
