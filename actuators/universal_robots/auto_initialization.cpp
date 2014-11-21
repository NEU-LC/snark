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

#include "auto_initialization.h"

namespace snark { namespace ur { namespace handlers {

result auto_initialization::run( started_reply_t started_update )
{
    std::cerr << name() << "command auto init" << std::endl;

    std::map< char, std::string > initj;
    initj[5] = "speedj_init([0,0,0,0,0,-0.1],0.05,0.04)";
    initj[4] = "speedj_init([0,0,0,0,0.1,0],0.05,0.0333333)";
    initj[3] = "speedj_init([0,0,0,-0.1,0,0],0.05,0.0266667)";
    initj[2] = "speedj_init([0,0,0.05,0,0,0],0.05,0.02)";
    initj[1] = "speedj_init([0,-0.05,0,0,0,0],0.05,0.0133333)";
    initj[0] = "speedj_init([0.05,0,0,0,0,0],0.05,0.00666667)";

    if( !status_.is_initialising_ready() ) {
        std::cerr << name() << "auto_init failed because robotic arm mode is " << status_.mode_str() << std::endl;
        return result( "cannot auto initialise robot if robot mode is not set to initializing", result::error::failure );
    }
    
    /// Signal that movement has started
    started_update();
    
    static const comma::uint32 retries = 50;
    // try for two joints right now
    bool stop_now = false;
    for( int joint_id=5; joint_id >=0 && !signaled_ && !stop_now; --joint_id )
    {

        stop_now = interrupt_();
        while( !signaled_ && !stop_now )
        {
            if( status_.jmode( joint_id ) != jointmode::initializing ) { break; }

            // move it a little bit
            os << initj[ joint_id ] << std::endl;
            os.flush();

            // wait tilt joint stopped
            for( std::size_t k=0; k<retries && !signaled_ && !stop_now; ++k ) 
            {
                /// Check and read any new input command from the user, if so we stop auto init.
                usleep( 0.005 * 1000000u );
                update_status_(); ///  This guarantee a status is read or an exception is thrown
                double vel = status_.velocities[ joint_id ];
                if( std::fabs( vel ) <= 0.03 ) break;
                stop_now = interrupt_();
            }
            
            if( signaled_ || stop_now ) { return result( "auto_init cancelled", result::error::cancelled ); }
        }

        if( status_.jmode( joint_id ) == jointmode::running ) {
            std::cerr << name() << "joint " << joint_id << " initialised" << std::endl;
            continue;
        }
        else 
        {
            if( signaled_ || stop_now ) { return result( "auto_init cancelled", result::error::cancelled ); }
            else
            {
                std::cerr << name() << "failed to initialise joint: " << joint_id << ", joint mode: " << status_.jmode_str( joint_id ) << std::endl;
                return result( "failed to auto initialise a joint", result::error::failure );
            }
       }
    }
    if( signaled_ || stop_now  ) {
        os << "speedj_init([0,0,0,0,0,0],0.05,0.0133333)" << std::endl;
        os.flush();
    }

    std::cerr << name() << "command auto init completed" << std::endl;
    return result();
}

} } } // namespace snark { namespace ur { namespace handlers {
