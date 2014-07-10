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

#include <gtest/gtest.h>
#include <comma/csv/stream.h>
#include <comma/csv/ascii.h>
#include <comma/visiting/traits.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include "../traits.h"
#include <boost/property_tree/json_parser.hpp>


namespace snark { namespace robot_arm {

template < typename T >
comma::csv::ascii< T >& ascii() { 
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}


TEST( robot_arm, robot_arm_config )
{
    config cfg;
    cfg.home_position[0] = 1.11;
    cfg.home_position[1] = 2.22322;
    cfg.home_position[5] = 6.0;

    std::string line;
    EXPECT_EQ( "1.11,2.22322,0,0,0,6", ascii< config >().put( cfg, line ) );

    const std::string expected = "{\n    \"home_position\":\n    {\n        \"0\": \"1.11\",\n        \"1\": \"2.22322\",\n        \"2\": \"0\",\n        \"3\": \"0\",\n        \"4\": \"0\",\n        \"5\": \"6\"\n    }\n}\n";
	{
        std::ostringstream oss;
        boost::property_tree::ptree t;
        comma::to_ptree to_ptree( t );
        comma::visiting::apply( to_ptree ).to( cfg );
        boost::property_tree::write_json( oss, t );    
        EXPECT_EQ( expected, oss.str() );
	}
	{
		std::istringstream iss( expected );
        boost::property_tree::ptree t;
        comma::from_ptree( t, true );
        boost::property_tree::read_json( iss, t );
        comma::from_ptree from_ptree( t, true );
        config conf;
        comma::visiting::apply( from_ptree ).to( conf );

        EXPECT_EQ( cfg.home_position[0], conf.home_position[0] );
        EXPECT_EQ( cfg.home_position[1], conf.home_position[1] );
        EXPECT_EQ( cfg, conf );
    }
}



} } // namespace snark { namespace robot_arm {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
