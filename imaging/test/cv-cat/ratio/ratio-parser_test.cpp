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

#include "../../../../imaging/cv_mat/ratio.h"

#include <gtest/gtest.h>

#include <boost/assign.hpp>

#include <iostream>

using namespace snark::cv_mat;
using boost::spirit::ascii::space;
typedef std::string::const_iterator iterator_type;

namespace
{

#if 0
    void usage( bool verbose = false )
    {
        std::cerr << comma::verbose.app_name() << " a ratio parser using Boost::Spirit\n";
        std::cerr << "\n";
        std::cerr << "usage: " << comma::verbose.app_name() << " [<options>] < stdin > stdout\n";
        std::cerr << "\n";
        std::cerr << "input:\n";
        std::cerr << "    either enter a string to parse (see below) or 'q' (or 'Q') to quit\n";
        std::cerr << "\n";
        std::cerr << "output:\n";
        std::cerr << "    the results of parsing (see below)\n";
        std::cerr << "\n";
        std::cerr << "options:\n";
        std::cerr << "    --help,-h:       show this help, --help --verbose for more help";
        std::cerr << "    --linear-combination: parse linear combination of the form\n";
        std::cerr << "            Ar + Bg + Cb + Da + F\n";
        std::cerr << "        here A,B,C,D,F are floating-point constants and r,g,b,a are channel names\n";
        std::cerr << "        a multiplication sign '*' is allowed between the constant and channel name\n";
        std::cerr << "        the F term gives a constant offset applied to the output\n";
        std::cerr << "\n";
        std::cerr << "        output in linear combination mode is a comma-separated list of coefficients\n";
        std::cerr << "        for the constant term and r,g,b,a channels (see the examples)\n";
        std::cerr << "\n";
        std::cerr << "    --ratio: parse ratios of linear combination (default); input is one of the forms\n";
        std::cerr << "            ( linear combination ) / ( linear combination )\n";
        std::cerr << "        here 'linear combination' is in the form Ar + Bg + Cb + Da + F, as above\n";
        std::cerr << "            term / ( linear combination )\n";
        std::cerr << "        here 'term' is either A or Ac, where A is a constant and c is one of channel names\n";
        std::cerr << "        for a single term, surrounding brackets are optional, 'term' is the same as '(term)'\n";
        std::cerr << "            ( linear combination ) / term\n";
        std::cerr << "            term / term\n";
        std::cerr << "            linear combination\n";
        std::cerr << "        here the linear combination becomes the ratio numerator, while the denominator is set to 1\n";
        std::cerr << "        brackets are optional around the numerator-only linear combination\n";
        std::cerr << "\n";
        std::cerr << "        output in ratio mode are two comma-separated lists of coefficients for the numerator\n";
        std::cerr << "        and denominator separated by '/' (see the examples)\n";
        std::cerr << "\n";
        std::cerr << "examples (outputs shown on the line below):\n";
        std::cerr << "    terms and signs between them may be separated by spaces\n";
        std::cerr << "        " << comma::verbose.app_name() << " --linear-combination 10 + 2r - 3g\n";
        std::cerr << "        10,2,-3,0,0\n";
        std::cerr << "    multiplication signs are optional and multipliers of 1 or -1 can be omitted\n";
        std::cerr << "        " << comma::verbose.app_name() << " --linear-combination 2*b - a\n";
        std::cerr << "        0,0,0,2,-1\n";
        std::cerr << "    leading '+' sign is ignored; arbitrary floating-point notation is supported\n";
        std::cerr << "        " << comma::verbose.app_name() << " --linear-combination +2*b - 1.3e-4r\n";
        std::cerr << "        0,-1.3e-4,0,2,0\n";
        std::cerr << "    order of channel terms does not matter\n";
        std::cerr << "        " << comma::verbose.app_name() << " --linear-combination 1 + r - g + b\n";
        std::cerr << "        1,1,-1,1,0\n";
        std::cerr << "        " << comma::verbose.app_name() << " --linear-combination b - g + 1 + r\n";
        std::cerr << "        1,1,-1,1,0\n";
        std::cerr << "    ratios are composed of linear combinations in the natural way; '--ratio' is optional (default)\n";
        std::cerr << "        " << comma::verbose.app_name() << " --ratio ( a +2*b - 3r ) / ( r + b )\n";
        std::cerr << "        0,-3,0,2,1/0,1,0,1,0\n";
        std::cerr << "        " << comma::verbose.app_name() << " a / ( r + b )\n";
        std::cerr << "        0,0,0,0,1/0,1,0,1,0\n";
        std::cerr << "        " << comma::verbose.app_name() << " (3a + 4*b) / g\n";
        std::cerr << "        0,0,0,4,3/0,0,1,0,0\n";
        std::cerr << "        " << comma::verbose.app_name() << " 3a / g\n";
        std::cerr << "        0,0,0,0,3/0,0,1,0,0\n";
        std::cerr << "    brackets are mandatory for multi-term linear combinations with one exception, numerator-only\n";
        std::cerr << "    input\n";
        std::cerr << "        " << comma::verbose.app_name() << " 3a + b / g\n";
        std::cerr << "        error ...\n";
        std::cerr << "        " << comma::verbose.app_name() << " 3a + b\n";
        std::cerr << "        0,0,0,1,3/1,0,0,0,0\n";
        std::cerr << std::endl;
        exit( 0 );
    }
#endif

    template< typename T >
    bool process( const ratios::parser< iterator_type, T > & parser
                , const std::string & input
                , T & result )
    {
        iterator_type begin = input.begin();
        iterator_type end = input.end();
        bool status = phrase_parse( begin, end, parser, space, result );
        return status && ( begin == end );
    }

    void verify( const ratios::combination & c, const std::vector< double > & expected )
    {
        EXPECT_EQ( expected.size(), ratios::channel::NUM_CHANNELS ); // self-check
        for ( int ci = ratios::channel::constant; ci != ratios::channel::NUM_CHANNELS ; ++ci )
        {
            EXPECT_EQ( c.terms[ci].value, expected[ci] );
        }
    }

} // namespace anonymous

TEST( ratio, term )
{
    ratios::rules< iterator_type > rules;
    ratios::parser< iterator_type, ratios::term > parser( rules.term_ );
    ratios::term v;

    {
        std::string input = "3b";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, 3.0 );
        EXPECT_EQ( v.c, ratios::channel::blue );
    }

    {
        std::string input = "-2* g";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, -2.0 );
        EXPECT_EQ( v.c, ratios::channel::green );
    }

    {
        std::string input = "r";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, 1.0 );
        EXPECT_EQ( v.c, ratios::channel::red );
    }

    {
        std::string input = "-2";
        EXPECT_TRUE( process( parser, input, v ) );
        EXPECT_EQ( v.value, -2.0 );
        EXPECT_EQ( v.c, ratios::channel::constant );
    }

    {
        std::string input = "2h";
        EXPECT_FALSE( process( parser, input, v ) );
    }

    {
        std::string input = "2 + g";
        EXPECT_FALSE( process( parser, input, v ) );
    }
}

TEST( ratio, combination )
{
    ratios::rules< iterator_type > rules;
    ratios::parser< iterator_type, ratios::combination > parser( rules.combination_ );
    ratios::combination v;

    {
        std::string input = "3b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of( 0.0 )( 0.0 )(0.0)(3.0)(0.0) );
        EXPECT_EQ( v.stringify(), "3b" );
    }

    {
        std::string input = "-3.5b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of( 0.0 )( 0.0 )(0.0)(-3.5)(0.0) );
        EXPECT_EQ( v.stringify(), "-3.5b" );
    }

    {
        std::string input = "1 + r - 3*a";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of( 1.0 )( 1.0 )(0.0)(0.0)(-3.0) );
        EXPECT_EQ( v.stringify(), "1 + r - 3a" );
    }

    {
        std::string input = "g - a + b";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of( 0.0 )( 0.0 )(1.0)(1.0)(-1.0) );
        EXPECT_EQ( v.stringify(), "g + b - a" );
    }

    {
        std::string input = "g - a + b/2";
        EXPECT_FALSE( process( parser, input, v ) );
    }

    {
        std::string input = ".0g";
        EXPECT_TRUE( process( parser, input, v ) );
        verify( v, boost::assign::list_of( 0.0 )( 0.0 )(0.0)(0.0)(0.0) );
        EXPECT_EQ( v.stringify(), "0" );
    }
}

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
