// This file is part of Ark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// Ark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Ark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with Ark. If not, see <http://www.gnu.org/licenses/>.

#include <comma/visiting/traits.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>

#include <boost/algorithm/string/predicate.hpp>

#include <type_traits>

namespace {

    bool verbose = false;

    const char* name = "image-color-calc: ";

    void usage( bool verbose = false )
    {
        std::cerr << std::endl;
        std::cerr << name << "perform conversion between RGB, YCbCr, and YPbPr color spaces on input streams." << std::endl;
        std::cerr << std::endl;
        std::cerr << "usage: cat input.bin | image-color-calc <operation> [<options>] > output.bin " << std::endl;
        std::cerr << std::endl;
        std::cerr << "operations" << std::endl;
        std::cerr << "  convert" << std::endl;
        std::cerr << "      conversion between RGB and YCbCr colorspaces" << std::endl;
        std::cerr << std::endl;
        std::cerr << "options (operation-dependent)" << std::endl;
        std::cerr << std::endl;
        std::cerr << "convert" << std::endl;
        std::cerr << "    --to=<colorspace>; default=YCbCr; direction of conversion, RGB, YCbCr, YPbPr (case-insensitive)" << std::endl;
        std::cerr << "    --from=<colorspace>; default=RGB; input colorspace, RGB or YCbCr (case-insensitive), opposite to direction of conversion" << std::endl;
    //    std::cerr << "    --header=[fields=<f>[;binary=<b>]]; input image is prepended with header that is omitted from conversion" << std::endl;
    //    std::cerr << "    --rows=<count>; default=1; the number of rows in the image" << std::endl;
    //    std::cerr << "    --cols,--columns=<count>; default=1; the number of columns in the image" << std::endl;
    //    std::cerr << "    --count,--size=<size>; default=1; the number of pixels in the image, rows times columns" << std::endl;
        std::cerr << std::endl;
        std::cerr << "general options" << std::endl;
        std::cerr << "    --binary=[<format>]: binary format of input stream" << std::endl;
        std::cerr << "    --format=[<format>]: format hint for ascii input (as coefficients for conversion depend on data type)" << std::endl;
        std::cerr << "    --fields=[<fields>]; default: operation-dependent" << std::endl;
        std::cerr << "    --flush; flush after every line or binary record" << std::endl;
        std::cerr << "    --input-fields; show default field names (depend on operation and direction of conversion) and exit" << std::endl;
        std::cerr << std::endl;
        std::cerr << "examples" << std::endl;
        std::cerr << "    convert" << std::endl;
        std::cerr << "        most simple, RGR to YCbCr (default), assume ub data" << std::endl;
        std::cerr << "            echo 1,2,3 | image-color-calc convert" << std::endl;
        std::cerr << "        input precision ambiguous, define explicitly; same default conversion applied" << std::endl;
        std::cerr << "            echo 1,0.2,0.3 | image-color-calc convert --format=3d" << std::endl;
        std::cerr << "        explicit conversion direction; '--to=RGB' would do the same" << std::endl;
        std::cerr << "            echo 1,0.2,0.3 | image-color-calc convert --from=YCbCr" << std::endl;
        std::cerr << "        handle binary, apply default conversion" << std::endl;
        std::cerr << "            echo 1,0.2,0.3 | csv-to-bin 3f | image-color-calc convert --binary=3f" << std::endl;
    //    std::cerr << "        input data have headers; skip header, apply default conversion to data" << std::endl;
    //    std::cerr << "            ... | image-color-calc convert --header=\"fields=id,t;binary=ui,t\" --rows=100 --cols=200" << std::endl;
    //    std::cerr << "        alternative form of the same" << std::endl;
    //    std::cerr << "            ... | image-color-calc convert --header=\"fields=id,t;binary=ui,t\" --count=$(( 100 * 200 ))" << std::endl;
    //    std::cerr << "        special fields in the header; rows, columns and format are taken from the header, cannot be given explicitly" << std::endl;
    //    std::cerr << "            ... | image-color-calc convert --header=\"fields=t,rows,cols,type;binary=t,3ui\"" << std::endl;
        std::cerr << "        using fields to select values to convert; RGB values 1,2,3 are converted to YCbCr" << std::endl;
        std::cerr << "            echo 'value',1,2,3,20170101T000000 | image-color-calc convert --fields=name,r,g,b,t" << std::endl;
        std::cerr << "        field names select conversion from YCbCr to RGB, precision explicitly hinted" << std::endl;
        std::cerr << "            echo 'value',1,2,3,20170101T000000 | image-color-calc convert --fields=name,y,cr,cb,t --format=,3ui," << std::endl;
        std::cerr << "        same example on binary data" << std::endl;
        std::cerr << "            echo 'value',1,2,3,20170101T000000 | csv-to-bin s[10],3ui,t | image-color-calc convert --fields=name,y,cr,cb,t --binary=s[10],3ui,t" << std::endl;
        std::cerr << "        using neutral field names to select values to convert and explicitly define conversion" << std::endl;
        std::cerr << "            echo 'value',1,2,3,20170101T000000 | image-color-calc convert --fields=name,channel0,channel1,channel2,t --to=ycbcr" << std::endl;
        std::cerr << std::endl;
        exit( 0 );
    }

    // enums do not have ctors, sigh
    struct colorspace
    {
        enum cspace {
            none = 0,
            rgb,
            ycbcr,
            ypbpr
        };

        cspace value;
        colorspace( const std::string & s ) : value( boost::iequals( s, "rgb" ) ? rgb : ( boost::iequals( s, "ycbcr" ) ? ycbcr : ( boost::iequals( s, "ypbpr" ) ? ypbpr : none ) ) ) { }
        colorspace( colorspace::cspace v ) : value( v ) { }

        operator std::string() const {
            switch( value ) {
                case( none ):  return "none";  break;
                case( rgb ):   return "rgb";   break;
                case( ycbcr ): return "ycbcr"; break;
                case( ypbpr ): return "ypbpr"; break;
            }
            return "none"; // avoid a warning
        };

        static std::vector< std::string > field_names( cspace c )
        {
            static std::map< cspace, std::vector< std::string > > m = {
                { rgb, comma::split( "r,g,b" , ',' ) },
                { ycbcr, comma::split( "y,cb,cr", ',' ) },
                { ypbpr, comma::split( "y,pb,pr", ',' ) },
                { none, comma::split( "channel0,channel1,channel2", ',' ) }
            };
            return m[ c ];
        }

        void set_to_opposite_if_not_given( const colorspace & other )
        {
            if ( value != colorspace::none ) { return; }
            switch ( other.value ) {
                case colorspace::rgb:    value = colorspace::ycbcr; break;
                case colorspace::ycbcr:  value = colorspace::rgb;   break;
                default:
                    COMMA_THROW( comma::exception, "cannot set the colorspace value to the opposite of " << std::string( other ) );
            }
        }
    };

    std::ostream & operator<<( std::ostream & os, const colorspace & c )
    {
        os << std::string( c );
        return os;
    }

    // do not call fields RGB because can contain YCbCr or any other color space
    template< typename T >
    struct pixel
    {
        T channel0;
        T channel1;
        T channel2;
        pixel( T c0 = 0, T c1 = 0, T c2 = 0 ) : channel0( c0 ), channel1( c1 ), channel2( c2 ) {}
    };

    template< typename i, bool i_is_float, typename o, bool o_is_float  >
    struct converter
    {
        static pixel< o > to_ycrcb( const pixel< i > & p );
    };

    template< typename i, typename o >
    pixel< o > to_ycrcb( const pixel< i > & p )
    {
        return converter< i, std::is_floating_point< i >::value, o, std::is_floating_point< o >::value >::to_ycrcb( p );
    };

    // float-to-float, double-to-double, and crosses
    template< typename i, typename o >
    struct converter< i, true, o, true >
    {
        static pixel< o > to_ycrcb( const pixel< i > & p )
        {
            typedef typename std::conditional< sizeof(o) <= sizeof(i), i, o >::type l;
            return pixel< o >( o(  l(0.299)    * l(p.channel0) + l(0.587)    * l(p.channel1) + l(0.114)    * l(p.channel2) )
                             , o( -l(0.168736) * l(p.channel0) - l(0.331264) * l(p.channel1) + l(0.5)      * l(p.channel2) )
                             , o(  l(0.5)      * l(p.channel0) - l(0.418688) * l(p.channel1) - l(0.081312) * l(p.channel2) ) );
        }
    };

    // floating-point-to-ub
    template< typename f >
    struct converter< f, true, unsigned char, false >
    {
        static pixel< unsigned char > to_ycrcb( const pixel< f > & p )
        {
            typedef unsigned char ub;
            return pixel< ub >(  16 + ub(  65.481 * p.channel0 + 128.553 * p.channel1 + 24.966 * p.channel2 )
                              , 128 + ub( -37.797 * p.channel0 - 74.203  * p.channel1 + 112.0  * p.channel2 )
                              , 128 + ub( 112.0   * p.channel0 - 93.786  * p.channel1 - 18.214 * p.channel2 ) );
        }
    };

    // to continue, ub-to-floating-point, longer integers (short, int)?

    template< typename i, typename o >
    int convert( const comma::csv::options & csv )
    {
        if ( csv.binary() )
        {
            #ifdef WIN32
            _setmode( _fileno( stdin ), _O_BINARY );
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
        }
        comma::csv::input_stream<  pixel< i > > is( std::cin, csv );
        comma::csv::options output_csv;
        output_csv.flush = csv.flush;
        if( csv.binary() ) { output_csv.format( comma::csv::format::value< pixel< o > >() ); }
        comma::csv::output_stream< pixel< o > > os( std::cout, output_csv );
        comma::csv::tied< pixel< i >, pixel< o > > tied( is, os );
        while( is.ready() || std::cin.good() )
        {
            const pixel< i > * p = is.read();
            if( !p ) { break; }
            tied.append( to_ycrcb< i, o >( *p ) );
            if ( output_csv.flush ) { std::cout.flush(); }
        }
        return 0;
    }

    bool fields_have_required( const std::vector< std::string > & fields, const std::vector< std::string > & required )
    {
        for ( const auto & r : required ) {
            if ( std::find( fields.begin(), fields.end(), r ) == fields.end() ) { return false; }
        }
        return true;
    }

    colorspace get_colorspace_from_fields( const std::vector< std::string > & fields, const std::vector< colorspace > & spaces )
    {
        if ( spaces.empty() ) { COMMA_THROW( comma::exception, "no colorspaces provided to choose from" ); }
        unsigned int total = 0;
        colorspace rv( colorspace::none );
        for ( const auto & c : spaces ) {
            if ( fields_have_required( fields, colorspace::field_names( c.value ) ) ) {
                ++total;
                rv = c;
            }
        }
        if ( total > 1 ) { COMMA_THROW( comma::exception, "contradictory field names match multiple colorspaces" ); }
        if ( total == 0 ) { COMMA_THROW( comma::exception, "field names do not match the requested colorspaces "
                                                           << std::accumulate( std::next( spaces.begin() ), spaces.end(), std::string( spaces.front() )
                                                                             , []( const colorspace & c, const std::string & a ) { return a + "," + std::string( c ); } ) ); }
        return rv;
    }

    void rename_fields_to_channels( std::vector< std::string > & fields, const colorspace & c )
    {
        const std::vector< std::string > & channels = colorspace::field_names( colorspace::none );
        const std::vector< std::string > & own = colorspace::field_names( c.value );
        std::map< std::string, std::string > map;
        std::transform( own.begin(), own.end(), channels.begin(), std::inserter( map, map.end() ), []( const std::string & k, const std::string & v ) { return std::make_pair( k, v ); } );
        for ( size_t i = 0; i < fields.size(); ++i ) { auto search = map.find( fields[i] ); if ( search != map.end() ) { fields[i] = search->second; } }
    }

    void setup_fields_for_colorspace( std::vector< std::string > & fields, const colorspace & c )
    {
        std::vector< colorspace > spaces( 1, c );
        if ( c.value != colorspace::none ) { spaces.push_back( colorspace::none ); }
        colorspace found = get_colorspace_from_fields( fields, spaces );
        if ( found.value != colorspace::none ) { rename_fields_to_channels( fields, c ); }
    }

} // anonymous

namespace comma { namespace visiting {

template < typename T > struct traits< pixel< T > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, pixel< T > & p, Visitor& v )
    {
        v.apply( "channel0", p.channel0 );
        v.apply( "channel1", p.channel1 );
        v.apply( "channel2", p.channel2 );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const pixel< T > & p, Visitor& v )
    {
        v.apply( "channel0", p.channel0 );
        v.apply( "channel1", p.channel1 );
        v.apply( "channel2", p.channel2 );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
#if 0
    {
        pixel< float > i(0.1, 0.2, 0.3);
        pixel< float > o = to_ycrcb< float, float >( i );
        comma::csv::output_stream< pixel< float > > os( std::cout );
        os.write( o );
    }
    {
        pixel< float > i(0.1, 0.2, 0.3);
        pixel< double > o = to_ycrcb< float, double >( i );
        comma::csv::output_stream< pixel< double > > os( std::cout );
        os.write( o );
    }
    {
        pixel< float > i(0.1, 0.2, 0.3);
        pixel< unsigned char > o = to_ycrcb< float, unsigned char >( i );
        comma::csv::output_stream< pixel< unsigned char > > os( std::cout );
        os.write( o );
    }
#endif
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        if ( !csv.binary() && options.exists( "--format" ) ) { csv.format( options.value< std::string >( "--format" ) ); }
        csv.full_xpath = true;
        verbose = options.exists("--verbose,-v");
        std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields", "--fields,-f,--binary,-b,--format,--to,--from");
        if( ops.empty() ) { std::cerr << name << "please specify an operation." << std::endl; return 1;  }
        if( ops.size() > 1 ) { std::cerr << name << "please specify only one operation, got " << comma::join( ops, ' ' ) << std::endl; return 1; }
        std::string operation = ops.front();
        std::vector< std::string > fields = comma::split( csv.fields, csv.delimiter );

        // the user may specify the direction of conversion by many ways
        // if --from is specified:
        //     if fields are not given, fields are set to the from-specific defaults
        //     if fields are given, the required fields must be present (and renamed if needed)
        // otherwise, if --fields is given, infer the from colorspace from fields
        // otherwise, if --to is given, infer the fields and from colorspace from --to
        // finally, if none of the above, apply defaults
        colorspace toc( options.value< std::string >( "--to", "none" ) );
        colorspace fromc( options.value< std::string >( "--from", "none" ) );
        if ( fromc.value != colorspace::none ) {
            if ( options.exists( "--fields,-f" ) )
            {
                setup_fields_for_colorspace( fields, fromc );
                csv.fields = comma::join( fields, ',' );
            } else {
                csv.fields = "channel0,channel1,channel2";
            }
            toc.set_to_opposite_if_not_given( fromc );
        } else {
            if ( options.exists( "--fields,-f" ) )
            {
                std::vector< colorspace > spaces = { colorspace( colorspace::rgb ), colorspace( colorspace::ycbcr ), colorspace( colorspace::ypbpr ) };
                fromc = get_colorspace_from_fields( fields, spaces );
                // now fromc cannot be none
                rename_fields_to_channels( fields, fromc );
                csv.fields = comma::join( fields, ',' );
                toc.set_to_opposite_if_not_given( fromc );
            } else {
                // '--from' not given, '--fields' not given, infer from '--to'
                switch ( toc.value ) {
                    case colorspace::rgb:   fromc.value = colorspace::ycbcr; break;
                    case colorspace::ycbcr: fromc.value = colorspace::rgb;   break;
                    case colorspace::ypbpr: fromc.value = colorspace::rgb;   break;
                    case colorspace::none:
                        // last resort, all defaults
                        fromc.value = colorspace::rgb;
                        toc.value = colorspace::ycbcr;
                        break;
                }
                csv.fields = "channel0,channel1,channel2";
            }
        }
        if ( csv.binary() ) {
            // assume all fields have same size
            size_t first_field = std::distance( fields.begin(), std::find( fields.begin(), fields.end(), "channel0" ) );
            comma::csv::format::element e = csv.format().offset( first_field );
            if ( fromc.value == colorspace::rgb && toc.value == colorspace::ycbcr ) {
                if ( e.type == comma::csv::format::float_t ) { return convert< float, unsigned char >( csv ); }
                if ( e.type == comma::csv::format::double_t ) { return convert< double, unsigned char >( csv ); }
                COMMA_THROW( comma::exception, "conversion from " << comma::csv::format::to_format( e.type ) << " RGB data is not supported" );
            }
            if ( fromc.value == colorspace::rgb && toc.value == colorspace::ypbpr ) {
                return convert< float, float >( csv );
                if ( e.type == comma::csv::format::float_t ) { return convert< float, float >( csv ); }
                if ( e.type == comma::csv::format::double_t ) { return convert< double, double >( csv ); }
                COMMA_THROW( comma::exception, "conversion from " << comma::csv::format::to_format( e.type ) << " RGB data is not supported" );
            }
        } else {
            // hope for the best
            std::cerr << name << "warning, assume ascii input is analog RGB (floating-point, 0-to-1)" << std::endl;
            if ( fromc.value == colorspace::rgb && toc.value == colorspace::ycbcr ) { return convert< float, unsigned char >( csv ); }
            if ( fromc.value == colorspace::rgb && toc.value == colorspace::ypbpr ) { return convert< float, float >( csv ); }
            COMMA_THROW( comma::exception, "not yet implemented" );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name << "unknown exception" << std::endl; }
    return 1;
}
