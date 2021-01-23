// Copyright (c) 2021 Vsevolod Vlaskine

#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/string/split.h>
#include "record.h"

namespace snark { namespace graphics { namespace plotting {

record record::sample( const std::string& fields, unsigned int size )
{
    plotting::record r;
    const auto& s = comma::split( fields.empty() ? std::string( "series[0]/x,series[0]/y" ) : fields, ',' ); // quick and dirty
    for( const auto& f: s ) // todo: check for time field
    {
        if( f == "series" ) // todo: quick and dirty, check fields are valid
        { 
            if( size == 0 ) { COMMA_THROW( comma::exception, "'series' field present, but number of series not specified for fields: \"" << fields << "\"" ); }
            r.series.resize( size );
            for( auto& t: r.series ) { t.x = 0; t.y = 0; } // quick and dirty
        }
        else if( f.substr( 0, 7 ) == "series[" ) // todo? quick and dirty; use proper deserialisation?
        {
            auto p = f.find_first_of( ']' );
            if( p == std::string::npos ) { COMMA_THROW( comma::exception, "malformed field \"" << f << "\" in fields: \"" << fields << "\"" ); }
            unsigned int i = boost::lexical_cast< unsigned int >( f.substr( 7, p - 7 ) );
            if( r.series.size() <= i ) { r.series.resize( i + 1 ); }
            if( p + 1 == f.size() ) { r.series[i].x = 0; r.series[i].y = 0; continue; } // quick and dirty
            if( f[ p + 1 ] != '/' || p + 2 == f.size() ) { COMMA_THROW( comma::exception, "malformed field \"" << f << "\" in fields: \"" << fields << "\"" ); }
            std::string n = f.substr( p + 2 );
            if( n == "x" ) { r.series[i].x = 0; }
            if( n == "y" ) { r.series[i].y = 0; }
            if( n == "z" ) { r.series[i].z = 0; }
        }
    }
    return r;
}
    
} } } // namespace snark { namespace graphics { namespace plotting {
