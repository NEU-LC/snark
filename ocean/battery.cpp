#include "battery.h"

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

hex_value_t::hex_value_t() : value(0)
{

}

hex_value_t::hex_value_t(comma::uint16 v) : value(v)
{

}


    
data_t::data_t() : address(0), value(0)
{

}

hex_data_t::hex_data_t() : controller_id(0), battery_id(0), values(6)
{
}



    
} } // namespace snark { namespace ocean {
