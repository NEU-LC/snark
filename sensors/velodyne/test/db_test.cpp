// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/archive/tmpdir.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <gtest/gtest.h>
#include <snark/sensors/velodyne/db.h>
#include <snark/sensors/velodyne/impl/angle.h>
#include <snark/sensors/velodyne/impl/serializable_db.h>
#include "./db.h"

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
            db db;
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

