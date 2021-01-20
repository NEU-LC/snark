// Copyright (c) 2021 Vsevolod Vlaskine

#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "record.h"

namespace snark { namespace graphics { namespace plotting {

record record::sample( const std::string& fields, unsigned int size )
{
    plotting::record r;
    const auto& s = comma::split( fields, ',', true );
    if( s.empty() ) { r.x = 0; r.y = 0; }
    for( const auto& f: s ) // quick and dirty
    {
        if( f == "x" ) { r.x = 0; }
        else if( f == "y" ) { r.y = 0; }
        else if( f == "z" ) { r.z = 0; }
        else if( f == "series" )
        { 
            if( size == 0 ) { COMMA_THROW( comma::exception, "'series' field present, but number of series not specified for fields: \"" << fields << "\"" ); }
            r.series.resize( size );
            for( auto& t: r.series ) { t.x = 0; t.y = 0; } // quick and dirty
        }
        else if( f == "series[" ) // todo? quick and dirty; use proper deserialisation?
        {
            auto p = f.substr( 7 ).find_first_not_of( ']' );
            if( p == std::string::npos ) { COMMA_THROW( comma::exception, "malformed field \"" << f << "\" in fields: \"" << fields << "\"" ); }
            unsigned int i = boost::lexical_cast< unsigned int >( f.substr( 7, p - 7  ) );
            if( r.series.size() < i ) { r.series.resize( i + 1 ); }
            if( p + 1 == f.size() ) { r.series[i].x = 0; r.series[i].y = 0; } // quick and dirty
            std::string n = f.substr( p + 1 );
            if( n == "x" ) { r.series[i].x = 0; }
            if( n == "y" ) { r.series[i].y = 0; }
            if( n == "z" ) { r.series[i].z = 0; }
        }
    }
    return r;
}
    
} } } // namespace snark { namespace graphics { namespace plotting {
