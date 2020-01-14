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

namespace ratios
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

    inline term abs( const term & t ) { return term( std::abs( t.value ), t.c ); }

    std::ostream & operator<<( std::ostream & o, const term & t );

    struct combination
    {
        const static double epsilon;

        explicit combination();
        combination( const term & t );
        void update( const term & t );
        size_t non_zero_term_count() const;
        bool unity() const;
        std::string to_string( ) const;
        void print( std::ostream & o ) const;

        std::vector< term > terms;
    };

    inline std::ostream & operator<<( std::ostream & o, const combination & c ) { c.print( o ); return o; }

    struct ratio
    {
        combination numerator;
        combination denominator;

        explicit ratio() { denominator.terms[0].value = 1.0; }
        explicit ratio( const std::vector< double > & n, const std::vector< double > & d );

        static size_t num_channels() { return channel::NUM_CHANNELS; }
        std::string to_string( ) const;
        static std::string describe_syntax( size_t offset = 0 );
    };

    inline std::ostream& operator<<( std::ostream & o, const ratio & r ) { o << r.numerator << "/" << r.denominator; return o; }

    template< typename Iterator >
    struct rules
    {
        rules();

        qi::rule< Iterator, ratios::channel(), ascii::space_type > channel_;
        qi::rule< Iterator, ratios::term(), ascii::space_type > term_;
        qi::rule< Iterator, ratios::combination(), ascii::space_type > combination_;
        qi::rule< Iterator, ratios::ratio(), ascii::space_type > ratio_;
    };

    template< typename Iterator, typename What >
    struct parser : qi::grammar< Iterator, What(), ascii::space_type >
    {
        parser( const qi::rule< Iterator, What(), ascii::space_type > & start ) : parser::base_type( start ) {}
    };

    typedef std::vector< std::pair< double, double > > coefficients;

} // namespace ratios

} }  // namespace snark { namespace cv_mat {
