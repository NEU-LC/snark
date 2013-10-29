#ifndef SNARK_COMMS_PCAN_MESSAGE_H_
#define SNARK_COMMS_PCAN_MESSAGE_H_

#include <boost/array.hpp>
#include <comma/base/types.h>

namespace snark { namespace pcan {

struct message
{
    comma::uint32 id;
    comma::uint32 size;
    boost::array< unsigned char, 8 > bytes;
    // etc
    
    // todo: static message from_pcan( const PCAN_MESSAGE& m );
    // todo: static PCAN_MESSAGE to_pcan( const message& m );
};
    
} } // namespace snark { namespace pcan {

#endif // SNARK_COMMS_PCAN_MESSAGE_H_
