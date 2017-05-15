#ifndef SNARK_ACTUATORS_WHEELS_TRAITS_H
#define SNARK_ACTUATORS_WHEELS_TRAITS_H

#include <comma/visiting/traits.h>
#include "wheel_command.h"

namespace comma { namespace visiting {

template <> struct traits< snark::wheels::limit >
{
    template < typename K, typename V > static void visit( const K& key, snark::wheels::limit& t, V& v )
    {
        v.apply( "min", t.min );
        v.apply( "max", t.max );
    }

    template < typename K, typename V > static void visit( const K& key, const snark::wheels::limit& t, V& v )
    {
        v.apply( "min", t.min );
        v.apply( "max", t.max );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_ACTUATORS_WHEELS_TRAITS_H
