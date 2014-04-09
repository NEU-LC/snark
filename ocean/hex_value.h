#ifndef SNARK_OCEAN_HEX_VALUE
#define SNARK_OCEAN_HEX_VALUE
#include <comma/base/types.h>
#include <boost/array.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

namespace snark { namespace ocean {
    
typedef unsigned char uint8;


/// Wraps a fundamental type so that is can be serialise/deserialised to/from hex CSV string
template < typename T >
struct hex_value_t
{
    hex_value_t() : value(0)
    {
        BOOST_STATIC_ASSERT_MSG( boost::is_integral< T >::value, " T must be integral type" );
        BOOST_STATIC_ASSERT_MSG( (!boost::is_same< T, unsigned char >::value), " T must not be unsigned char, use unsigned short instead" );
        BOOST_STATIC_ASSERT_MSG( (!boost::is_same< T, char >::value), " T must not be char, use short instead" );

    }
    hex_value_t( T v ) : value(v)
    {

    }
    comma::uint16 byte() { return comma::uint16( value & 0x000000ff ); }
    
    T operator()() { return value; }
    
    T value;
};

/// For lexical_cast
template < typename T >
std::ostream& operator<<( std::ostream& ostream, const hex_value_t< T >& val )
{
    ostream << std::hex << val.value;
    return ostream;
}

/// For lexical_cast
template < typename T >
std::istream& operator>>( std::istream& istream, hex_value_t< T >& val )
{
    istream >> std::hex >> val.value;
    return istream;
}

/// A pair of data ( 1 byte) address and value ( 2 bytes )
/// See universal Smart Battery Specification doc
struct data_t
{
    data_t() : address( 0 ), value( 0 ) {}
    hex_value_t< comma::uint16 > address;
    hex_value_t< comma::uint16 > value;
};

/// Store values pushed from the Ocean controller
/// N is the number of data value pairs
template < int N >
struct hex_data_t
{
    hex_data_t() : controller_id(0), battery_id(0) {} 
    
    static const char battery_char = 'B';
    static const char setup_char = 'S';
    uint8 controller_id;   // packed controller ID and battery ID
    uint8 battery_id;
    std::string id_packed;
    /// Pairs of address identifying data and raw value
    boost::array< data_t, N > values;
};

} } // namespace snark { namespace ocean {

#endif //  SNARK_OCEAN_HEX_VALUE