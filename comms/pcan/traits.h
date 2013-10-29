#ifndef SNARK_COMMS_PCAN_TRAITS_H_
#define SNARK_COMMS_PCAN_TRAITS_H_

#include <comma/visiting/traits.h>
#include <snark/comms/pcan/message.h>

namespace comma { namespace visiting {
    
template <> struct traits< snark::pcan::message >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::pcan::message& p, Visitor& v )
    {
        v.apply( "id", p.id );
        v.apply( "size", p.size );
        v.apply( "bytes", p.bytes );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::pcan::message& p, Visitor& v )
    {
        v.apply( "id", p.id );
        v.apply( "size", p.size );
        v.apply( "bytes", p.bytes );
    }
};
    
} } // namespace comma { namespace visiting {

#endif // SNARK_COMMS_PCAN_TRAITS_H_
