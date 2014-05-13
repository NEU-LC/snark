#ifndef SNARK_OCEAN_TRAITS_H
#define SNARK_OCEAN_TRAITS_H
#include <comma/visiting/visit.h>
#include <comma/string/string.h>
#include <boost/lexical_cast.hpp>
#include "battery.h"
#include "hex_value.h"

namespace boost {
    
/// Make hex_value_t a fundamental type so far as visiting is concerned
/// It is lexical_cast able from/to hex string
template < typename T > struct is_fundamental< snark::ocean::hex_value_t< T > > : public true_type {};

}

namespace comma { namespace visiting {
using snark::ocean::data_t;
using snark::ocean::hex_value_t;
using snark::ocean::hex_data_t;
using snark::ocean::uint8;
using snark::ocean::battery_t;
using snark::ocean::controller_t;


template <> struct traits< data_t >
{
    template< typename K, typename V > static void visit( const K& k, data_t& t, V& v )
    {
        v.apply("address", t.address);
        v.apply("value", t.value);
    }

    template< typename K, typename V > static void visit( const K& k, const data_t& t, V& v )
    {
        v.apply("address", t.address );
        v.apply("value", t.value );
    }
};

template < int N > struct traits< boost::array< int, N > >
{
    template< typename K, typename V > static void visit( const K& k, boost::array< int, N >& t, V& v )
    {
        for( std::size_t i=0; i<N; ++i ) { v.apply( boost::lexical_cast< std::string >( i ), t[i] ); }
    }

    template< typename K, typename V > static void visit( const K& k, const boost::array< int, N >& t, V& v )
    {
        for( std::size_t i=0; i<N; ++i ) { v.apply( boost::lexical_cast< std::string >( i ), t[i] ); }
    }
};
    
template < int N > struct traits< hex_data_t< N > >
{
    template< typename K, typename V > static void visit( const K& k, hex_data_t< N >& t, V& v )
    {
        v.apply("controller_id", t.id_packed);
        t.battery_id = boost::lexical_cast< comma::uint16 >( t.id_packed[2] );
        t.controller_id = boost::lexical_cast< comma::uint16 >( t.id_packed[1] );
        v.apply("values", t.values);
    }

    template< typename K, typename V > static void visit( const K& k, const hex_data_t< N >& t, V& v )
    {
        v.apply("id_packed", t.id_packed );
        v.apply("values", t.values);
    }
};

template <> struct traits< battery_t >
{
    template< typename K, typename V > static void visit( const K& k, const battery_t& t, V& v )
    {
        v.apply("id", int(t.id) );
        v.apply("voltage", t.voltage.value() );
        v.apply("current", t.current.value() );
        v.apply("avg_current", t.avg_current.value() );
        v.apply("temperature", t.temperature.value() );
        v.apply("remaining_capacity", t.remaining_capacity.value() );
        v.apply("chargePc", t.chargePc );
        v.apply("time_to_empty", t.time_to_empty.total_seconds() / 3600.0 );
    }
};

template < int N > struct traits< controller_t< N > >
{
    template< typename K, typename V > static void visit( const K& k, const controller_t< N >& t, V& v )
    {
        v.apply("id", int(t.id) );
        v.apply("state", battery_t::state_to_string(t.state) );
        v.apply("total_power", t.total_power.value() );
        v.apply("total_current", t.total_current.value() );
        v.apply("avg_voltage", t.avg_voltage.value() );
        v.apply("batteries", t.batteries );
        v.apply("avgCharge", t.avgCharge );
    }
};   


} } // namespace comma { namespace visiting {

#endif // SNARK_OCEAN_TRAITS_H
