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

template < typename T >
hex_value_t< T >::hex_value_t() : value(0)
{
    typedef unsigned char uchar_;
    BOOST_STATIC_ASSERT_MSG( boost::is_integral< T >::value, " T must be integral type" );
    BOOST_STATIC_ASSERT_MSG( (!boost::is_same< T, uchar_ >::value), " T must not be unsigned char, use unsigned short instead" );
    BOOST_STATIC_ASSERT_MSG( (!boost::is_same< T, char >::value), " T must not be char, use short instead" );

}

template < typename T >
hex_value_t< T >::hex_value_t(T v) : value(v)
{

}


    
data_t::data_t() : address(0), value(0)
{

}

hex_data_t::hex_data_t() : controller_id(0), battery_id(0), values(6)
{
}

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


template class hex_value_t< comma::uint16 >;
    
} } // namespace snark { namespace ocean {
