#include "battery.h"
#include <boost/static_assert.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

namespace snark { namespace ocean {
    
    
battery_t::battery_t() : id(0), chargePc(-999)
{

}
battery_t::battery_t( uint8 id_ ) : id( id_ ), chargePc(-999) {}


template < int N >
controller_t< N >::controller_t() : id(0), state( state_t::unknown ), avgCharge(-999)
{

}
template < int N >
controller_t< N >::controller_t( uint8 id_ ) : id( id_ ), state( state_t::unknown ), avgCharge(-999) {}


void battery_t::operator&(const data_t& data)
{
    switch( data.address() )
    {
        case address::temperature:
        {
            static const double unit = 0.1; // Kelvin
            temperature = data.value() * unit * kelvin; // 0.1k unit
            break;
        }
        case address::voltage:
        {
            voltage = data.value() / 1000.0 * volt; // millivolts to volts
            break;
        }
        case address::current:
        {
            current = data.value() / 1000.0 * current; //mAmp to Amps
            break;
        }
        case address::avg_current:
        {
            avg_current = data.value() / 1000.0 * current; //mAmp to Amps
            break;
        }
        case address::remaining_capacity:
        {
            remaining_capacity = data.value() / 100.0 * watt; // unit is 10mWh
        }
        case address::rel_state_of_charge:
        {
            chargePc = data.value();    // percentage, unit is %
            break;
        }
        case address::run_time_to_empty:
        {
            time_to_empty = boost::posix_time::minutes( data.value() );
        }
        default:
        {
            return;
        }
    }
}

// void controller_t::operator&(const data_t& data)
// {
//     switch( data.address() )
//     {
//         case address::run_time_to_empty:
//         {
//             static const double unit = 0.1; // Kelvin
//             temperature = data.value() * unit * kelvin;
//             break;
//         }
//         default:
//         {
//             return;
//         }
//     }
// }


// template < int N >
// void controller_t< N >::operator&(const hex_data_t< comma::uint16 >& line_data)
// {
//     
// 
// }
template class controller_t< 3 >;



} } // namespace snark { namespace ocean {
