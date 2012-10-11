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

