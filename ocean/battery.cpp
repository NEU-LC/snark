#include "battery.h"
#include <boost/static_assert.hpp>

namespace snark { namespace ocean {
    
    
battery_t::battery_t() : id(0), chargePc(-999)
{

}
battery_t::battery_t( uint8 id_ ) : id( id_ ), chargePc(-999) {}


template < int N >
controller_t< N >::controller_t() : id(0), avgCharge(-999)
{

}
template < int N >
controller_t< N >::controller_t( uint8 id_ ) : id( id_ ), avgCharge(-999) {}

} } // namespace snark { namespace ocean {
