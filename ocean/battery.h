#ifndef SNARK_OCEAN_BATTERY_H
#define SNARK_OCEAN_BATTERY_H
#include <comma/base/types.h>
#include <boost/graph/graph_concepts.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/array.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "hex_value.h"
#include "units.h"

namespace snark { namespace ocean {

struct address {
    enum { temperature = 0x08, voltage = 0x09, current = 0x0a, avg_current=0x0b, rel_state_of_charge=0x0d, remaining_capacity=0x0f,
           run_time_to_empty=0x11
    };
    
};

struct battery_t
{
    battery_t();
    battery_t( uint8 id );
    uint8 id;
    voltage_t voltage;
    current_t current;
    current_t avg_current;
    temperature_t temperature;  
    power_t remaining_capacity;
    double chargePc; // Charge percentage
    boost::posix_time::time_duration time_to_empty;
    
    void operator&( const data_t& data );
    
    template < int P >
    void operator&( const hex_data_t< P >& line_data )
    {
        for( typename boost::array< data_t, P >::const_iterator it=line_data.values.begin(); it!=line_data.values.end(); ++it ) { *this & ( *it ); } 
    }
};

template < int N >
struct controller_t
{
    struct state_t {
        enum { AC, FC, FD, NG };
    };
    
    controller_t();
    controller_t( uint8 id_ );
    uint8 id;
    int state;
    boost::array< battery_t, N > batteries;
    typedef typename boost::array< battery_t, N >::const_iterator const_iter;
    
    
    power_t total_power;
    current_t total_current;
    voltage_t avg_voltage;
    double avgCharge; // percentage
    
    void operator&( const data_t& data );
    
    /// Populate the controller and controller's batteries with hex data ( pushed data from controller )
    template < int P >
    void operator&( const hex_data_t< P >& line_data )
    {
        if( line_data.controller_id != id ) { 
            return; 
        }
        if( line_data.battery_id > N ) 
        {
            std::cerr << "battery " << line_data.battery_id  << "is not on controller " << int(id) << std::endl;
            return;
        }
        
        batteries[ line_data.battery_id - 1 ] & line_data;
        
//         for( typename hex_data_t< P >::value_iter it=line_data.begin(); it!=line_data.end(); ++it ) { *this & ( *it ); } 
    }
    /// Get consolidated values from the batteries, e.g. total power or total current
    void consolidate()
    {
        total_current = 0*ampere;
        total_power = 0*watt;
        avg_voltage = 0*volt;
        avgCharge = 0;
        for( const_iter it=batteries.begin(); it!=batteries.end(); ++it ) 
        { 
            total_current += it->avg_current;
            total_power += it->remaining_capacity;
            avg_voltage == it->voltage;
            avgCharge += it->chargePc;
        } 
        //avg_voltage = ( avg_voltage.value()/N ) * volt;
        avg_voltage /= N;
        avgCharge /= N;
    }
};

} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_BATTERY_H
