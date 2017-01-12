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

#include <iostream>
#include <string>

#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/bind.hpp>

namespace snark{ namespace cv_mat {

namespace ratio
{
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;
    namespace phoenix = boost::phoenix;

    struct channel {
        enum channel_types {
            constant = 0,
            red,
            green,
            blue,
            alpha,
            NUM_CHANNELS
        };
        channel_types c;
        channel( channel_types c_ = NUM_CHANNELS ) : c( c_ ) {}
        channel & operator=( channel_types c_ ) { c = c_; return *this; }
        operator channel_types() const { return c; }
    };

    std::ostream & operator<<( std::ostream & o, const channel & c );

    struct term
    {
        double value;
        channel c;
        term( double v_ = 0.0, channel c_ = channel::NUM_CHANNELS ) : value( v_ ), c( c_ ) {}
    };

    std::ostream & operator<<( std::ostream & o, const term & t );

    struct combination
    {
        const static double epsilon;

        combination();

        void update( const term & t );

        // pretty-printer
        std::string stringify( ) const;

        void print( std::ostream & o ) const;

        std::vector< term > terms;
    };

    std::ostream & operator<<( std::ostream & o, const combination & c )
    {
        c.print( o );
        return o;
    }

    template <typename Iterator>
    struct combination_parser : qi::grammar< Iterator, ratio::combination(), ascii::space_type >
    {
        combination_parser() : combination_parser::base_type( start )
        {
            using qi::double_;
            using qi::_1;
            using qi::lit;
            using ascii::space;
            using phoenix::ref;
            using phoenix::bind;
            using qi::_val;
            using qi::eps;

            channel = eps[ _val = ratio::channel() ] >> ( lit('r')[ _val = channel::red ] | lit('g')[ _val = channel::green ] | lit('b')[ _val = channel::blue ] | lit('a')[ _val = channel::alpha ] );
            term = eps[ _val = ratio::term( 1.0, channel::constant) ] >>
                (
                        double_[ bind( &term::value, _val ) = _1 ] >> -lit('*') >> -channel[ bind( &term::c, _val ) = _1 ]
                    |   channel[ bind( &term::c, _val ) = _1 ]
                    |   lit('+')[ bind( &term::value, _val ) = 1 ] >> -( double_[ bind( &term::value, _val ) *= _1 ] >> -lit('*') ) >> channel[ bind( &term::c, _val ) = _1 ]
                    |   lit('-')[ bind( &term::value, _val ) = -1 ] >> -( double_[ bind( &term::value, _val ) *= _1 ] >> -lit('*') ) >> channel[ bind( &term::c, _val ) = _1 ]
                );
            start = eps[ _val = ratio::combination() ] >> *( term[ bind( &combination::update, _val, _1 ) ] );
        }

        qi::rule< Iterator, ratio::channel(), ascii::space_type > channel;
        qi::rule< Iterator, ratio::term(), ascii::space_type > term;
        qi::rule< Iterator, ratio::combination(), ascii::space_type > start;
    };

} // namespace ratio

} }  // namespace snark { namespace cv_mat {
