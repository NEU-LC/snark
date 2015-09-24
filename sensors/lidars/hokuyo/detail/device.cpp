#include "device.h"
#include <comma/base/exception.h>
#include <comma/io/publisher.h>
#include "../message.h"

namespace snark { namespace hokuyo {

turn_laser_on::turn_laser_on(stream_base& ios, laser_device& device, bool reboot_on_error):ios(ios)
{
    state_command start( "BM" ); // starts transmission
    state_reply start_reply;

    ios.write( start.data(), state_command::size  );
    ios.flush();
    
    comma::io::select select;
    select.read().add( ios.native() );
    
    select.wait( 1 ); // wait one select for reply, it can be much smaller
    if( !select.read().ready( ios.native() ) ) { 
        COMMA_THROW( comma::exception, "no reply received from laser scanner after a startup (BM) command: " << std::string( start.data(), state_command::size ) ); 
    }
    ios.read( start_reply.data(), state_reply::size  );
    
    if( start_reply.status() != 0 && 
        start_reply.status() != 10 &&
        start_reply.status() != 2 ) // 0 = success, 2 seems to be returned when it is already in scanning mode but idle
    {
        if( reboot_on_error )
        {
            device.reboot(ios);
        }
        COMMA_THROW( comma::exception, std::string("Starting laser with BM command failed, status: ") + std::string( start_reply.status.data(), 2 ) ); 
    }
}

/// On exit just send a QT command, although it does not seem to be needed.
turn_laser_on::~turn_laser_on()
{
    const state_command stop( "QT" );
    ios.write( stop.data(), state_command::size );
    ios.flush();
    ios.close();
}

} }  // namespace snark { namespace hokuyo {
    
