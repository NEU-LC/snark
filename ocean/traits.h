#ifndef SNARK_OCEAN_TRAITS_H
#define SNARK_OCEAN_TRAITS_H
#include <comma/visiting/visit.h>
#include <comma/string/string.h>
#include <boost/lexical_cast.hpp>
#include "battery.h"

namespace boost {
    
/// Make hex_value_t a fundamental type so far as visiting is concerned
/// It is lexical_cast able from/to hex string
template <> struct is_fundamental< snark::ocean::hex_value_t > : public true_type {};

}

namespace comma { namespace visiting {
using snark::ocean::data_t;
using snark::ocean::hex_value_t;
using snark::ocean::hex_data_t;
using snark::ocean::uint8;


template <> struct traits< data_t >
{
    template< typename K, typename V > static void visit( const K& k, data_t& t, V& v )
    {
//         std::string addr, val;
//         v.apply("address", addr);
//         v.apply("value", val);
//         t.address = boost::lexical_cast< uint8 >( addr );
//         t.value = boost::lexical_cast< comma::uint16 >( addr );
        v.apply("address", t.address);
        v.apply("value", t.value);
    }

    template< typename K, typename V > static void visit( const K& k, const data_t& t, V& v )
    {
        v.apply("address", t.address );
        v.apply("value", t.value );
    }
};
    
template <> struct traits< hex_data_t >
{
    template< typename K, typename V > static void visit( const K& k, hex_data_t& t, V& v )
    {
        v.apply("controller_id", t.id_packed);
        v.apply("values", t.values);
    }

    template< typename K, typename V > static void visit( const K& k, const hex_data_t& t, V& v )
    {
        v.apply("id_packed", boost::lexical_cast< std::string >( t.id_packed ) );
        v.apply("values", t.values);
    }
};
    
} } // namespace comma { namespace visiting {

#endif // SNARK_OCEAN_TRAITS_H
