#include "auto_initialization.h"

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

namespace arm = robotic_arm;

void auto_initialization::read_status()
{
    comma::uint32 sleep_usec = 0.03 * 1000000u; 
    unsigned char loops = 3;

    for( std::size_t i=0; i<loops; ++i )
    {
        select_.check();

        if( select_.read().ready( iss_.fd() ) )
        {
            iss_->read( status_.data(), fixed_status::size );
            // read all buffered data
            while( iss_->rdbuf()->in_avail() > 0 ) { iss_->read( status_.data(), fixed_status::size ); }

            return;
        }

        usleep( sleep_usec );
    }
    std::cerr << name() << "failed to read status in auto init" << std::endl;
    COMMA_THROW( comma::exception, "failed to read status in auto_init" );
}



result auto_initialization::run()
{
    std::cerr << name() << "command auto init" << std::endl;

    std::map< char, std::string > initj;
    // snark-ur10-from-console: Initialising joint (5)
    initj[5] = "speedj_init([0,0,0,0,0,-0.1],0.05,0.04)";
    // snark-ur10-from-console: Initialising joint (4)
    initj[4] = "speedj_init([0,0,0,0,-0.1,0],0.05,0.0333333)";
    // snark-ur10-from-console: Initialising joint (3)
    initj[3] = "speedj_init([0,0,0,-0.1,0,0],0.05,0.0266667)";
    // snark-ur10-from-console: Initialising joint (2)
    initj[2] = "speedj_init([0,0,0.05,0,0,0],0.05,0.02)";
    // snark-ur10-from-console: Initialising joint (1)
    initj[1] = "speedj_init([0,-0.05,0,0,0,0],0.05,0.0133333)";
    // snark-ur10-from-console: Initialising joint (0)
    initj[0] = "speedj_init([0.05,0,0,0,0,0],0.05,0.00666667)";

    if( status_.mode() != robotmode::initializing ) {
        std::cerr << name() << "auto_init failed because robotic arm mode is " << status_.mode_str() << std::endl;
        return result( "cannot auto initialise robot if robot mode is not set to initializing", result::error::failure );
    }

    static const comma::uint32 retries = 50;
    // try for two joints right now
    for( int joint_id=5; joint_id >=0 && !signaled_ && inputs_.is_empty(); --joint_id )
    {

        while( !signaled_ && inputs_.is_empty() )
        {
            if( status_.joint_modes[joint_id]() != jointmode::initializing ) { break; }

            os << initj[ joint_id ] << std::endl;
            os.flush();

            // std::cerr << "joint " << joint_id << " is in initializing " << std::endl;
            std::cerr.flush();

            // wait tilt joint stopped
            for( std::size_t k=0; k<retries; ++k ) 
            {
                usleep( 0.01 * 1000000u );
                // std::cerr << "reading status" << std::endl;
                read_status();

                double vel = status_.velocities[ joint_id ]();
                if( std::fabs( vel ) <= 0.03 ) break;
            }
            
            /// Check and read any new input commands
            inputs_.read();
        }

        // todo check force also
        if( status_.jmode( joint_id ) == jointmode::running ) {
            std::cerr << name() << "joint " << joint_id << " initialised" << std::endl;
            continue;
        }
        else 
        {
            std::cerr << name() << "failed to initialise joint: " << joint_id 
                      << ", joint mode: " << status_.jmode_str( joint_id ) << std::endl;
            return result( "failed to auto initialise a joint", result::error::failure );
        }
    }
    if( signaled_  ) {
        os << "speedj_init([0,0,0,0,0,0],0.05,0.0133333)" << std::endl;
        os.flush();
    }


    std::cerr << name() << "command auto init completed" << std::endl;
    return result();
}



} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
