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


#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/archive/tmpdir.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <gtest/gtest.h>
#include "../impl/angle.h"
#include "../impl/serializable_db.h"
#include "db.h"

#include <boost/date_time/posix_time/posix_time.hpp>

namespace snark {  namespace velodyne {

TEST(db, db)
{
    impl::serializable_db DB;
    try
    {
        {
            std::ostringstream oss;
            boost::archive::xml_oarchive oa( oss );
            oa << BOOST_SERIALIZATION_NVP( DB );
            std::istringstream iss( oss.str() );
            EXPECT_TRUE( iss.good() );
            boost::archive::xml_iarchive ia( iss );
            ia >> BOOST_SERIALIZATION_NVP( DB );
        }
        {
            hdl64::db db;
            std::istringstream iss( velodyne::test::db_string );
            assert( iss.good() );
            iss >> db;
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "unknown exception" << std::endl;
    }
}

} } // namespace snark {  namespace velodyne {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}

