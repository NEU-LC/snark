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

#include <type_traits>
#include <boost/static_assert.hpp>

// todo
// - <operation> (for now only one operation: convert)  DONE
// - remove --format
// - implement something like: image-color-calc convert --from rgb,0-255 --to ypbpr
//                                                      --from rgb,uw --to ypbpr,d
//                                                      --from rgb,f --to ypbpr
//   support ub, etc as shorthand
// - on error, print erroneous values
// - pixel: use vector instead of channel0, channel1, etc or pixel type templated by colourspace with array for channels

namespace {

    bool verbose = false;
    
    const char* name = "image-color-calc: ";

    void usage( bool verbose = false )
    {
        std::cerr << std::endl;
        std::cerr << name << "perform various color transformations on input images" << std::endl;
        std::cerr << std::endl;
        std::cerr << "usage: cat input.csv | image-color-calc <operation> [<options>] > output.csv" << std::endl;
        std::cerr << std::endl;
        std::cerr << "operations:" << std::endl;
        std::cerr << "    convert" << std::endl;
        std::cerr << "        perform conversion between rgb, ycbcr, ypbpr, and other colorspaces on input streams" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        usage:" << std::endl;
        std::cerr << "            cat input.csv | image-color-calc convert --from <colorspace>[,<type>] --to <colorspace>[,<type>,<format>]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        <colorspace>" << std::endl;
        std::cerr << "            rgb:    red-green-blue, 0 to 255 in ub, 0 to 1 in floating-point, etc." << std::endl;
        std::cerr << "            ycbcr:  digital luma and chroma, 16-235 in y, 16-240 in cb,cr" << std::endl;
        std::cerr << "            ypbpr:  analog luma and chroma, 0 to 1 floating-point" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        <type>" << std::endl;
        std::cerr << "            describes the range of values; not the same as storage format" << std::endl;
        std::cerr << "                ub:  from 0 to 255" << std::endl;
        std::cerr << "                uw:  from 0 to 65535" << std::endl;
        std::cerr << "                ui:  from 0 to 4294967295" << std::endl;
        std::cerr << "                f:   from 0 to 1; storage format cannot be an integer" << std::endl;
        std::cerr << "                d:   from 0 to 1; storage format cannot be an integer" << std::endl;
        std::cerr << "            default types are colorspace-specific:" << std::endl;
        std::cerr << "                rgb:    ub" << std::endl;
        std::cerr << "                ycbcr:  ub (only part of the 0-255 range is used due to footroom/headroom)" << std::endl;
        std::cerr << "                ypbpr:  f" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        output <type>,<format>" << std::endl;
        std::cerr << "            by default, output is double-precision values in the range of the \"to\" <colorspace>, e.g., from 0. to 255. for rgb" << std::endl;
        std::cerr << "            use <type> to rescale to different range; by default, values would be stored in variable of that <type>" << std::endl;
        std::cerr << "            use <format> to specify different storage, e.g." << std::endl;
        std::cerr << "                --to rgb,uw:   convert to rgb in 0-65535 range, truncate value, store as 4-byte integer" << std::endl;
        std::cerr << "                --to rgb,uw,d: convert to rgb in 0-65535 range, store as doubles, keep precision" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    options" << std::endl;
        std::cerr << "        --from=[<colorspace>[,<type>]]; input colorspace and type; colorspace can be also inferred from fields" << std::endl;
        std::cerr << "        --to=<colorspace>[,<type>]; destination colorspace, mandatory, and its optional type" << std::endl;
        std::cerr << "        --input-fields; show input field names for the given --from <colorspace> and exit" << std::endl;
        std::cerr << "        --input-type=[<type>]; the type of input values; use when input <colorspace> is inferred from fields" << std::endl;
        std::cerr << "        --output-fields; show output field names for the given --to <colorspace> and exit" << std::endl;
        std::cerr << "        --output-type=[<type>]; alternative way to specify output type, provided for symmetry with --input-type" << std::endl;
        std::cerr << std::endl;
        std::cerr << "options" << std::endl;
        std::cerr << "    --help,-h; print this message; --help --verbose: print more help" << std::endl;
        std::cerr << "    --verbose,-v; more output" << std::endl;
        std::cerr << std::endl;
        std::cerr << "csv options" << std::endl;
        if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; } else { std::cerr << "    run --help --verbose for details..." << std::endl << std::endl; }
        std::cerr << "examples" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    convert" << std::endl;
        std::cerr << "        rgb to ycbcr; explicit type to define input as 8-bit digital values" << std::endl;
        std::cerr << "            echo 1,2,3 \\" << std::endl;
        std::cerr << "                | image-color-calc convert --from rgb,ub --to ycbcr" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        same direction but input is analog, a value from 0 to 1" << std::endl;
        std::cerr << "            echo 1,0.2,0.3 \\" << std::endl;
        std::cerr << "                | image-color-calc convert --from rgb,f --to ycbcr" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        handle binary, same conversion as above; note that output is in doubles by default" << std::endl;
        std::cerr << "            echo 1,0.2,0.3 | csv-to-bin 3f \\" << std::endl;
        std::cerr << "                | image-color-calc convert --from=rgb,f --to=ycbcr \\" << std::endl;
        std::cerr << "                | csv-from-bin 3f,3d" << std::endl;
        std::cerr << std::endl;
        if ( verbose ) {
            std::cerr << "        same as above but rescale and truncate output to short integers" << std::endl;
            std::cerr << "            echo 1,0.2,0.3 | csv-to-bin 3f \\" << std::endl;
            std::cerr << "                | image-color-calc convert --from=rgb,f --to=ycbcr,uw \\" << std::endl;
            std::cerr << "                | csv-from-bin 3f,3uw" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        same as above but rescale output to short integers without truncation and loosing precision" << std::endl;
            std::cerr << "            echo 1,0.2,0.3 | csv-to-bin 3f \\" << std::endl;
            std::cerr << "                | image-color-calc convert --from=rgb,f --to=ycbcr,uw,d \\" << std::endl;
            std::cerr << "                | csv-from-bin 3f,3d" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        using fields to select values to convert, no --from needed but have to specify input type explicitly" << std::endl;
            std::cerr << "            echo 'value',128,128,128,20170101T000000 \\" << std::endl;
            std::cerr << "                | image-color-calc convert --fields=name,r,g,b,t --input-type=ub --to=ycbcr" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        field names select conversion from ycbcr; inputs are read as doubles, using ub range of values (default for ycbcr)" << std::endl;
            std::cerr << "            echo 'value',30.5,40.2,50.3,20170101T000000 \\" << std::endl;
            std::cerr << "                | image-color-calc convert --fields=name,y,cb,cr,t --to rgb" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        same example on binary data; input is read as floats interpreted as values in uw range" << std::endl;
            std::cerr << "            echo 'value',3000.5,4000.2,5000.3,20170101T000000 | csv-to-bin s[10],3f,t \\" << std::endl;
            std::cerr << "                | image-color-calc convert --fields=name,y,cb,cr,t --binary=s[10],3f,t --input-type=uw --to rgb,ub\\" << std::endl;
            std::cerr << "                | csv-from-bin s[10],3f,t,3ub" << std::endl;
            std::cerr << std::endl;
            std::cerr << "        using neutral field names to select values to convert and explicitly define conversion" << std::endl;
            std::cerr << "            echo 'value',1,2,3,20170101T000000 \\" << std::endl;
            std::cerr << "                | image-color-calc --fields=name,channel0,channel1,channel2,t --from rgb,ub --to=ycbcr,ui,d" << std::endl;
            std::cerr << std::endl;
        } else {
            std::cerr << "    use --help --verbose for more examples" << std::endl;
        }
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
        colorspace( const std::string & s ) : value( s == "rgb" ? rgb : ( s == "ycbcr" ? ycbcr : ( s == "ypbpr" ? ypbpr : none ) ) ) { }
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
    };

    std::ostream & operator<<( std::ostream & os, const colorspace & c )
    {
        os << std::string( c );
        return os;
    }

    // do not call fields rgb because can contain ycbcr or any other colorspace
    template< typename T >
    struct pixel
    {
        T channel0;
        T channel1;
        T channel2;
        pixel( T c0 = 0, T c1 = 0, T c2 = 0 ) : channel0( c0 ), channel1( c1 ), channel2( c2 ) {}
    };

    // general template, never defined
    // keep the precision explicit because some of the colorspaces may exist in multiple formats
    // (most are 1-to-1, so this is excessive in general)
    template< colorspace::cspace ic, typename it, colorspace::cspace oc, typename ot  >
    struct converter
    {
        static pixel< ot > to( const pixel< it > & p );
    };

    // analog YPbPr from analog RGB (float-to-float, double-to-double, and crosses)
    template< typename it, typename ot >
    struct converter< colorspace::rgb, it, colorspace::ypbpr, ot >
    {
        static pixel< ot > to( const pixel< it > & p )
        {
            BOOST_STATIC_ASSERT( std::is_floating_point< it >::value );
            BOOST_STATIC_ASSERT( std::is_floating_point< ot >::value );
            typedef typename std::conditional< sizeof(ot) <= sizeof(it), it, ot >::type l;
            return pixel< ot >( ot(  l(0.299)    * l(p.channel0) + l(0.587)    * l(p.channel1) + l(0.114)    * l(p.channel2) )
                              , ot( -l(0.168736) * l(p.channel0) - l(0.331264) * l(p.channel1) + l(0.5)      * l(p.channel2) )
                              , ot(  l(0.5)      * l(p.channel0) - l(0.418688) * l(p.channel1) - l(0.081312) * l(p.channel2) ) );
        }
    };

    // digital YCbCr from analog RGB (floating-point-to-ub)
    template< typename it >
    struct converter< colorspace::rgb, it, colorspace::ycbcr, unsigned char >
    {
        static pixel< unsigned char > to( const pixel< it > & p )
        {
            typedef unsigned char ub;
            auto rub = []( it v ){ return ub( std::round( v ) ); };
            return pixel< ub >(  16 + rub(  65.481 * p.channel0 + 128.553 * p.channel1 +  24.966 * p.channel2 )
                              , 128 + rub( -37.797 * p.channel0 -  74.203 * p.channel1 + 112.0   * p.channel2 )
                              , 128 + rub( 112.0   * p.channel0 -  93.786 * p.channel1 -  18.214 * p.channel2 ) );
        }
    };

    // digital YCbCr from digital RGB (ub-to-ub)
    template< >
    struct converter< colorspace::rgb, unsigned char, colorspace::ycbcr, unsigned char >
    {
        static pixel< unsigned char > to( const pixel< unsigned char > & p )
        {
            typedef unsigned char ub;
            auto rub = []( double v ){ return ub( std::round( v ) ); };
            return pixel< ub >( rub(  16 + (  65.738 * p.channel0 + 129.057 * p.channel1 +  25.064 * p.channel2 ) / 256. )
                              , rub( 128 + ( -37.945 * p.channel0 -  74.494 * p.channel1 + 112.439 * p.channel2 ) / 256. )
                              , rub( 128 + ( 112.439 * p.channel0 -  94.154 * p.channel1 -  18.258 * p.channel2 ) / 256. ) );
        }
    };

    // digital RGB from digital YCbCr (ub-to-ub)
    template< >
    struct converter< colorspace::ycbcr, unsigned char, colorspace::rgb, unsigned char >
    {
        static pixel< unsigned char > to( const pixel< unsigned char > & p )
        {
            typedef unsigned char ub;
            auto rub = []( double v ){ return ub( std::round( v ) ); };
            return pixel< ub >( rub( 255/219. * ( p.channel0 - 16 )                                                     + 255/112.*0.701             * ( p.channel2 - 128 ) )
                              , rub( 255/219. * ( p.channel0 - 16 ) - 255/112.*0.886*0.114/0.587 * ( p.channel1 - 128 ) - 255/112.*0.701*0.299/0.587 * ( p.channel2 - 128 ) )
                              , rub( 255/219. * ( p.channel0 - 16 ) + 255/112.*0.886             * ( p.channel1 - 128 )                                                     ) );
        }
    };

    template< colorspace::cspace ic, typename it, colorspace::cspace oc, typename ot >
    void convert( const comma::csv::options & csv )
    {
        comma::csv::input_stream<  pixel< it > > is( std::cin, csv );
        comma::csv::options output_csv;
        output_csv.flush = csv.flush;
        if( csv.binary() ) { output_csv.format( comma::csv::format::value< pixel< ot > >() ); }
        comma::csv::output_stream< pixel< ot > > os( std::cout, output_csv );
        comma::csv::tied< pixel< it >, pixel< ot > > tied( is, os );
        while( is.ready() || std::cin.good() )
        {
            const pixel< it > * p = is.read();
            if( !p ) { break; }
            tied.append( converter< ic, it, oc, ot >::to( *p ) );
            if ( output_csv.flush ) { std::cout.flush(); }
        }
    }

    void from_rgb( const colorspace & toc, comma::csv::format::types_enum it, const comma::csv::options & csv )
    {
        switch( toc.value ) {
            case colorspace::ycbcr:
                if ( it == comma::csv::format::uint8 ) { convert< colorspace::rgb, unsigned char, colorspace::ycbcr, unsigned char >( csv ); return; }
                if ( it == comma::csv::format::float_t ) { convert< colorspace::rgb, float, colorspace::ycbcr, unsigned char >( csv ); return; }
                if ( it == comma::csv::format::double_t ) { convert< colorspace::rgb, double, colorspace::ycbcr, unsigned char >( csv ); return; }
                COMMA_THROW( comma::exception, "conversion from " << comma::csv::format::to_format( it ) << " rgb to " << toc << " is not supported" );
            case colorspace::ypbpr:
                if ( it == comma::csv::format::float_t ) { convert< colorspace::rgb, float, colorspace::ypbpr, float >( csv ); return; }
                if ( it == comma::csv::format::double_t ) { convert< colorspace::rgb, double, colorspace::ypbpr, double >( csv ); return; }
                COMMA_THROW( comma::exception, "conversion from " << comma::csv::format::to_format( it ) << " rgb to " << toc << " is not supported" );
            default:
                COMMA_THROW( comma::exception, "conversion from rgb to " << toc << " is not implemented yet" );
        }
    }

    void from_ycbcr( const colorspace & toc, const comma::csv::options & csv )
    {
        switch ( toc.value ) {
            case colorspace::rgb:
                convert< colorspace::ycbcr, unsigned char, colorspace::rgb, unsigned char >( csv ); return;
            default:
                COMMA_THROW( comma::exception, "conversion from ycbcr to " << toc << " is not implemented yet" );
        }
    }

    // the methods below are for parsing the command line

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
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        csv.full_xpath = true;
        verbose = options.exists("--verbose,-v");
        std::vector< std::string > unnamed = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields,--output-fields", "--fields,-f,--binary,-b,--format,--to,--from");
        if( 1 != unnamed.size() ) { std::cerr << name << "cannot extract the operation from the command-line arguments '" << options.string() << "'" << std::endl; return 1;  }

        const std::string & operation = unnamed[0];
        if ( operation == "convert" )
        {
            // the user may specify the input for conversion by two ways
            // if --from is specified:
            //     if fields are not given, fields are set to the from-specific defaults
            //     if fields are given, the required fields must be present (and renamed if needed)
            // otherwise, if --fields is given, infer the from colorspace from fields
            colorspace fromc( options.value< std::string >( "--from", "none" ) );
            if ( options.exists( "--input-fields" ) ) { std::cout << comma::join( colorspace::field_names( fromc.value ), ',' ) << std::endl; return 0; }
            colorspace toc( options.value< std::string >( "--to", "none" ) );
            if ( toc.value == colorspace::none ) { COMMA_THROW( comma::exception, "must provide destination colorspace using '--to'" ); }
            if ( options.exists( "--output-fields" ) ) { std::cout << comma::join( colorspace::field_names( toc.value ), ',' ) << std::endl; return 0; }
            std::vector< std::string > fields = comma::split( csv.fields, csv.delimiter );
            if ( fromc.value != colorspace::none ) {
                if ( options.exists( "--fields,-f" ) )
                {
                    setup_fields_for_colorspace( fields, fromc );
                } else {
                    fields = { "channel0", "channel1", "channel2" };
                }
                csv.fields = comma::join( fields, ',' );
            } else {
                if ( options.exists( "--fields,-f" ) )
                {
                    std::vector< colorspace > spaces = { colorspace( colorspace::rgb ), colorspace( colorspace::ycbcr ), colorspace( colorspace::ypbpr ) };
                    fromc = get_colorspace_from_fields( fields, spaces );
                    // now fromc cannot be none
                    rename_fields_to_channels( fields, fromc );
                    csv.fields = comma::join( fields, ',' );
                } else {
                    COMMA_THROW( comma::exception, "neither '--from' nor '--fields' are given, cannot determine the input colorspace" );
                }
            }

            // the actual processing is done below
            if ( verbose ) { std::cerr << name << "convert from " << fromc << " to " << toc << " colorspace using fields '" << comma::join( fields, ',' ) << "'" << std::endl; }
            switch ( fromc.value ) {
                case colorspace::rgb:
                    {
                        if ( !csv.binary() && !options.exists( "--format" ) ) { COMMA_THROW( comma::exception, "must supply '--format' for ASCII rgb inputs" ); }
                        const comma::csv::format & format = csv.binary() ? csv.format() : comma::csv::format( options.value< std::string >( "--format" ) );
                        // assume all input fields have same size
                        size_t first_field = std::distance( fields.begin(), std::find( fields.begin(), fields.end(), "channel0" ) );
                        from_rgb( toc, format.offset( first_field ).type, csv );
                    }
                    break;
                case colorspace::ycbcr:
                    from_ycbcr( toc, csv );
                    break;
                default:
                    COMMA_THROW( comma::exception, "conversion from " << fromc << " to " << toc << " is not implemented yet" );
            }
            return 0;
        } else {
            std::cerr << name << "unknown operation '" << operation << "', not one of: convert" << std::endl;
            return 1;
        }
    }
    catch( std::exception & ex ) { std::cerr << name << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name << "unknown exception" << std::endl; }
    return 1;
}
