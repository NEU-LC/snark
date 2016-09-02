#include "realsense.h"
#include <comma/csv/traits.h>
#include <boost/lexical_cast.hpp>

namespace comma { namespace visiting {
   

template <> struct traits< snark::realsense::stream_args >
{
    template< typename K, typename V > static void visit( const K& k, snark::realsense::stream_args& p, V& v )
    {
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        std::string f;
        v.apply( "format", f );
        p.format=snark::realsense::format(f);
        v.apply( "framerate", p.framerate );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::realsense::stream_args& p, V& v )
    {
        v.apply( "width", p.width );
        v.apply( "height", p.height );
        std::string f=boost::lexical_cast<std::string>(int(p.format.value));
        v.apply( "format", f );
        v.apply( "framerate", p.framerate );
    }
};

    
} } // namespace comma { namespace visiting {
