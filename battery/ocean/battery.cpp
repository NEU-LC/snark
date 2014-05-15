#include "battery.h"
#include <boost/static_assert.hpp>

namespace snark { namespace ocean {
    
std::string battery_t::state_to_string( int st ) 
{
    switch( st )
    {
        case battery_state::initialised:
            return "IN";
            break;
        case battery_state::uninitialised:
            return "UN";
            break;
        case battery_state::fully_discharged:
            return "FD";
            break;
        case battery_state::fully_charged:
            return "FC";
            break;
        case battery_state::discharging:
            return "DC";
            break;
        default:
            return "CH";
            break;

    }
}
// Removes checksum wrappers, TODO throws exception on incorrect checksum
std::string& battery_t::strip( std::string& line )
{
    /// '$B15,....,FF00%B2' becomes B15,....,FF00
    //std::size_t pos = ( line[ line.size() - 3 ] == '%' ? line.size()-4 : std::string::npos );
    std::size_t pos = line.find_first_of( '%', line.size() - 4 );
    if( pos != std::string::npos ) { --pos; }
    //std::cerr << "char: " << line[ line.size() - 3 ] << std::endl;
    line = line.substr( 1, pos);
    return line;
}

void battery_t::operator&(const data_t& data)
{
    // std::cerr << " address " << data.address() << std::endl;
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
            current = data.value.cast() / 1000.0 * ampere; //mAmp to Amps
            // std::cerr << "got current: " << current.value() << std::endl;
            break;
        }
        case address::average_current:
        {
            average_current = data.value.cast() / 1000.0 * ampere; //mAmp to Amps
            break;
        }
        case address::remaining_capacity:
        {
            remaining_capacity = data.value.cast() / 100.0 * watt; // unit is 10mWh
        }
        case address::rel_state_of_charge:
        {
            charge_pc = data.value();    // percentage, unit is %
            break;
        }
        case address::run_time_to_empty:
        {
            time_to_empty = boost::posix_time::minutes( data.value() );
        }
        case address::status:
        {
            if( !(data.value() &  battery_state::initialised) ) 
            {
                state = battery_state::uninitialised;
                return;
            }
            comma::uint16 val = data.value() & 0x0070;  // masks out everything including 'initialised' flag
            switch( val )
            {
                case battery_state::discharging:
                    state = battery_state::discharging;
                    break;
                case battery_state::fully_charged:
                    state = battery_state::fully_charged;
                    break;
                case battery_state::fully_discharged:
                    state = battery_state::fully_discharged;
                    break;
                default:
                    state = battery_state::charging;
                    break;
            }
            // std::cerr << "battery: " << int(id) <<  " state: " << state << " value: " << data.value() << " val: " << val <<std::endl;
            break;
        }
        default:
        {
            return;
        }
    }
}
    
} } // namespace snark { namespace ocean {
