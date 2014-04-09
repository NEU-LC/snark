#ifndef SNARK_OCEAN_BATTERY_H
#define SNARK_OCEAN_BATTERY_H
#include "units.h"
#include <comma/base/types.h>
#include <boost/graph/graph_concepts.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/array.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include "hex_value.h"

namespace snark { namespace ocean {


struct battery_t
{
    battery_t();
    battery_t( uint8 id );
    uint8 id;
    volt_t voltage;
    current_t current;
    temperature_t temperature;  
    double chargePc; // Charge percentage
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
    state_t state;
    boost::array< battery_t, N > batteries;
    
    
    power_t total_power;
    current_t total_current;
    boost::posix_time::time_duration time_to_empty;
    double avgCharge; // percentage
};

} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_BATTERY_H
