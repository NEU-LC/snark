#ifndef SNARK_OCEAN_BATTERY_H
#define SNARK_OCEAN_BATTERY_H
#include <comma/base/types.h>
#include <comma/base/exception.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/array.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "hex_value.h"
#include "units.h"
#include "commands.h"

namespace snark { namespace ocean {
    
static const int OCEAN_NAN = -999;

struct address {
     enum { temperature = 0x08, voltage = 0x09, current = 0x0a, average_current=0x0b, rel_state_of_charge=0x0d, remaining_capacity=0x0f,
            run_time_to_empty=0x11, status=0x16
     };
    
};
struct battery_state {
        enum { charging=0x00, fully_discharged=0x10, fully_charged=0x20, discharging=0x40, initialised=0x80, uninitialised };
};

struct battery_t
{

    static std::string state_to_string( int st );

    uint8 id;
    voltage_t voltage;
    current_t current;
    current_t average_current;
    temperature_t temperature;  
    power_t remaining_capacity;
    double charge_pc; // Charge percentage
    boost::posix_time::time_duration time_to_empty;
    int state;
    
    // void operator&( const data_t& data );
    
    template < int P >
    void operator&( const hex_data_t< P >& line_data )
    {
        for( typename boost::array< data_t, P >::const_iterator it=line_data.values.begin(); it!=line_data.values.end(); ++it ) { *this & ( 
*it ); } 
    }

    battery_t() : id(0), charge_pc( OCEAN_NAN ), state( battery_state::uninitialised ) {}
    battery_t( uint8 id_ ) : id( id_ ), charge_pc( OCEAN_NAN ), state( battery_state::uninitialised ) {}

    // update battery with new data
    void operator&(const data_t& data);
    // Removes checksum wrappers, TODO throws exception on incorrect checksum
    static std::string& strip( std::string& line );
};



template < int N >
struct controller
{
    uint8 id;
    int state;
    boost::array< battery_t, N > batteries;
    typedef typename boost::array< battery_t, N >::const_iterator const_iter;
    power_t total_power;
    current_t total_current;
    voltage_t average_voltage;
    double average_charge; // percentage

    static const char battery_data_char = 'B';
    static const char controller_data_char = 'C';

    controller() : id(0), state( battery_state::uninitialised ), average_charge( OCEAN_NAN ) 
    { 
        // One controller to 8 batteries only
        BOOST_STATIC_ASSERT_MSG( ( N > 0 && N <= 8 ), " T must be integral type" );
        set_battery_id(); 
        
    }
    controller( uint8 id_ ) : id( id_ ), state( battery_state::uninitialised ), average_charge( OCEAN_NAN ) { set_battery_id(); }

    void set_battery_id()
    {
        for( std::size_t i=0; i<=N; ++i ) { batteries[i].id = i + 1; }
    }
    
    void operator&( const data_t& data );
    
    /// Populate the controller and controller's batteries with hex data ( pushed data from controller )
    template < int P >
    void operator&( const hex_data_t< P >& line_data )
    {
        // std::cerr << "controller update: " << int( line_data.controller_id ) << " - Battery: " <<
        //     int( line_data.battery_id ) << std::endl;
        if( line_data.controller_id != id ) {  return;  }
        if( line_data.battery_id > N ) 
        {
            std::ostringstream ss;
            ss << "battery " << line_data.battery_id  << "is not on controller " << int(id) << std::endl;
            COMMA_THROW( comma::exception, ss.str() );
        }

        batteries[ line_data.battery_id - 1 ] & line_data;
    }
    /// Get consolidated values from the batteries, e.g. total power or total current
    void consolidate()
    {
        total_current = 0*ampere;
        total_power = 0*watt;
        average_voltage = 0*volt;
        average_charge = 0;
        for( const_iter it=batteries.begin(); it!=batteries.end(); ++it ) 
        { 
            total_current += it->average_current;
            total_power += it->remaining_capacity;
            average_voltage += it->voltage;
            average_charge += it->charge_pc;
        } 
        //average_voltage = ( average_voltage.value()/N ) * volt;
        average_voltage /= N;
        average_charge /= N;
        state = batteries.front().state;
    }
};

} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_BATTERY_H
