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


} } // namespace snark { namespace ocean {

#endif //  SNARK_OCEAN_HEX_VALUE