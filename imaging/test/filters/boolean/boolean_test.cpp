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

#include "../../../../imaging/cv_mat/detail/boolean.h"

#include <gtest/gtest.h>

#include <boost/assign.hpp>

#include <iostream>

using namespace snark::cv_mat;
using namespace snark::cv_mat::boolean;
using boost::spirit::ascii::space;

TEST( boolean, parser )
{
    const std::vector< std::string > inputs =
        {
            "a and b",
            "a or b",
            "a xor b",
            "not a",
            "not a and b",
            "not (a and b)",
            "a or b or c",
            "(a and b) xor ((c and d) or (a and b))",
            "a and b xor (c and d or a and b)",
        };
    const std::vector< std::string > expected =
        {
            "(a & b)",
            "(a | b)",
            "(a ^ b)",
            "(!a)",
            "((!a) & b)",
            "(!(a & b))",
            "(a | (b | c))",
            "((a & b) ^ ((c & d) | (a & b)))",
            "((a & b) ^ ((c & d) | (a & b)))",
        };
    for ( size_t i = 0; i < inputs.size(); ++i )
    {
        auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
        parser< decltype( f ) > p;

        expr result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        EXPECT_TRUE( ok );
        std::ostringstream os;
        os << result;
        EXPECT_EQ( os.str(), expected[i] );
        EXPECT_EQ( f, l );
    }
}
