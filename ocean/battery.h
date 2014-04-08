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

namespace snark { namespace ocean {

typedef unsigned char uint8;

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

/// Wraps a fundamental type so that is can be serialise/deserialised to/from hex CSV string
template < typename T >
struct hex_value_t
{
    hex_value_t();
    hex_value_t( T v );
    comma::uint16 byte() { return comma::uint16( value & 0x000000ff ); }
    
    T value;
};


/// A pair of data ( 1 byte) address and value ( 2 bytes )
/// See universal Smart Battery Specification doc
struct data_t
{
    data_t();
    hex_value_t< comma::uint16 > address;
    hex_value_t< comma::uint16 > value;
};


struct hex_data_t
{
    hex_data_t();
    
    static const char battery_char = 'B';
    static const char setup_char = 'S';
    uint8 controller_id;   // packed controller ID and battery ID
    uint8 battery_id;
    std::string id_packed;
    /// Pairs of address identifying data and raw value
    std::vector< data_t > values;
};

/// For lexical_cast
template < typename T >
std::ostream& operator<<( std::ostream& ostream, const hex_value_t< T >& val );

/// For lexical_cast
template < typename T >
std::istream& operator>>( std::istream& istream, hex_value_t< T >& val );

} } // namespace snark { namespace ocean {

#endif // SNARK_OCEAN_BATTERY_H
