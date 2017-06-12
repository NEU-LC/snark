// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "../color/colorspace.h"
#include "../color/pixel.h"
#include "../color/convert.h"
#include "../color/traits.h"

#include <comma/csv/stream.h>

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
        std::cerr << "                --to rgb,uw:   convert to rgb in 0-65535 range, truncate value, store as 2-byte integer" << std::endl;
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

#if 0
    template< typename T >
    pixel< T, ub >::pixel( T c0 = 0, T c1 = 0, T c2 = 0 ) : channel0( limits< T, ub >::trim( c0 ) ), channel1( limits< T, ub >::trim( c1 ) ), channel2( limits< T, ub >::trim( c2 ) ) {}

    template< typename T >
    pixel< T, uw >::pixel( T c0 = 0, T c1 = 0, T c2 = 0 ) : channel0( limits< T, uw >::trim( c0 ) ), channel1( limits< T, uw >::trim( c1 ) ), channel2( limits< T, uw >::trim( c2 ) ) {
        static_assert( !std::is_same< typename range_traits< ub >::value, T >::value, "cannot store value of uw range in ub variable" );
    }

    template< typename T >
    pixel< T, ui >::pixel( T c0 = 0, T c1 = 0, T c2 = 0 ) : channel0( limits< T, ui >::trim( c0 ) ), channel1( limits< T, ui >::trim( c1 ) ), channel2( limits< T, ui >::trim( c2 ) ) {
        static_assert( !std::is_same< typename range_traits< ub >::value, T >::value, "cannot store value of ui range in ub variable" );
        static_assert( !std::is_same< typename range_traits< uw >::value, T >::value, "cannot store value of ui range in uw variable" );
    }

    template< typename T >
    pixel< T, f >::pixel( T c0 = 0, T c1 = 0, T c2 = 0 ) : channel0( limits< T, f >::trim( c0 ) ), channel1( limits< T, f >::trim( c1 ) ), channel2( limits< T, f >::trim( c2 ) ) {
        static_assert( !std::is_same< typename range_traits< ub >::value, T >::value, "cannot store value of f range in ub variable" );
        static_assert( !std::is_same< typename range_traits< uw >::value, T >::value, "cannot store value of f range in uw variable" );
        static_assert( !std::is_same< typename range_traits< ui >::value, T >::value, "cannot store value of f range in ui variable" );
    }

    // general template, never defined
    // keep the precision explicit because some of the colorspaces may exist in multiple formats
    // (most are 1-to-1, so this is excessive in general)
    template< colorspace::cspace ic, typename it, colorspace::cspace oc, typename ot  >
    struct converter
    {
        static pixel< ot > to( const pixel< it > & p );
    };

    // analog YPbPr from analog RGB (float-to-float, double-to-double, and crosses)
    // rgb,*,f -> ypbpr,*,f
    // rgb,*,d -> ypbpr,*,d
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
    // rgb,*,f -> ycbcr,*,ub
    // rgb,*,d -> ycbcr,*,ub
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
    // rgb,*,ub -> ycbcr,*,ub
    // other conversions:
    // rgb,*,* -> rgb,d,* -> rgb,d,ub -> ycbcr,d,ub -> ycbcr,*,ub
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
    // ycbcr,*,ub -> rgb,*,ub
    // other conversions:
    // ycbcr,*,* -> ycbcr,d,* -> ycbcr,d,ub -> rgb,d,ub -> rgb,*,ub
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
#endif

    // the methods below are for parsing the command line

    bool fields_have_required( const std::vector< std::string > & fields, const std::vector< std::string > & required )
    {
        for ( const auto & r : required ) {
            if ( std::find( fields.begin(), fields.end(), r ) == fields.end() ) { return false; }
        }
        return true;
    }

    snark::imaging::colorspace get_colorspace_from_fields( const std::vector< std::string > & fields, const std::vector< snark::imaging::colorspace > & spaces )
    {
        if ( spaces.empty() ) { COMMA_THROW( comma::exception, "no colorspaces provided to choose from" ); }
        unsigned int total = 0;
        snark::imaging::colorspace rv( snark::imaging::colorspace::none );
        for ( const auto & c : spaces ) {
            if ( fields_have_required( fields, snark::imaging::colorspace::field_names( c.value ) ) ) {
                ++total;
                rv = c;
            }
        }
        if ( total > 1 ) { COMMA_THROW( comma::exception, "contradictory field names match multiple colorspaces" ); }
        if ( total == 0 ) { COMMA_THROW( comma::exception, "field names do not match the requested colorspaces "
                                                           << std::accumulate( std::next( spaces.begin() ), spaces.end(), std::string( spaces.front() )
                                                                             , []( const snark::imaging::colorspace & c, const std::string & a ) { return a + "," + std::string( c ); } ) ); }
        return rv;
    }

    void rename_fields_to_channels( std::vector< std::string > & fields, const snark::imaging::colorspace & c )
    {
        const std::vector< std::string > & channels = snark::imaging::colorspace::field_names( snark::imaging::colorspace::none );
        const std::vector< std::string > & own = snark::imaging::colorspace::field_names( c.value );
        std::map< std::string, std::string > map;
        std::transform( own.begin(), own.end(), channels.begin(), std::inserter( map, map.end() ), []( const std::string & k, const std::string & v ) { return std::make_pair( k, v ); } );
        for ( size_t i = 0; i < fields.size(); ++i ) { auto search = map.find( fields[i] ); if ( search != map.end() ) { fields[i] = search->second; } }
    }

    void setup_fields_for_colorspace( std::vector< std::string > & fields, const snark::imaging::colorspace & c )
    {
        std::vector< snark::imaging::colorspace > spaces( 1, c );
        if ( c.value != snark::imaging::colorspace::none ) { spaces.push_back( snark::imaging::colorspace::none ); }
        snark::imaging::colorspace found = get_colorspace_from_fields( fields, spaces );
        if ( found.value != snark::imaging::colorspace::none ) { rename_fields_to_channels( fields, c ); }
    }

} // anonymous

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

            // parsing origin
            snark::imaging::colorspace fromc( snark::imaging::colorspace::none );
            boost::optional< snark::imaging::range > fromr;
            const std::string & froms( options.value< std::string >( "--from", "" ) );
            if ( !froms.empty() )
            {
                const std::vector< std::string > & fromv = comma::split( froms, ',' );
                if ( fromv.size() > 2 ) { COMMA_THROW( comma::exception, "--from takes at most two comma-separated value" ); }
                fromc = snark::imaging::colorspace( fromv.front() );
                if ( fromv.size() == 2 ) { fromr = snark::imaging::stringify::to( fromv[1] ); }
            }
            // alternatively, get origin from fields
            std::vector< std::string > fields = comma::split( csv.fields, csv.delimiter );
            if ( fromc.value != snark::imaging::colorspace::none ) {
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
                    std::vector< snark::imaging::colorspace > spaces = { snark::imaging::colorspace( snark::imaging::colorspace::rgb )
                                                                       , snark::imaging::colorspace( snark::imaging::colorspace::ycbcr )
                                                                       , snark::imaging::colorspace( snark::imaging::colorspace::ypbpr ) };
                    fromc = get_colorspace_from_fields( fields, spaces );
                    // now fromc cannot be none
                    rename_fields_to_channels( fields, fromc );
                    csv.fields = comma::join( fields, ',' );
                } else {
                    COMMA_THROW( comma::exception, "neither '--from' nor '--fields' are given, cannot determine the input colorspace" );
                }
            }
            if ( !fromr ) { fromr = snark::imaging::stringify::to( snark::imaging::colorspace::default_range( fromc.value ) ); }
            if ( options.exists( "--input-fields" ) ) { std::cout << comma::join( snark::imaging::colorspace::field_names( fromc.value ), ',' ) << std::endl; return 0; }

            // parsing destination
            const std::string & tos = options.value< std::string >( "--to" );
            const std::vector< std::string > & tov = comma::split( tos, ',' );
            if ( tov.size() > 3 ) { COMMA_THROW( comma::exception, "--to takes at most three comma-separated value" ); }
            snark::imaging::colorspace toc = snark::imaging::colorspace( tov.front() );
            if ( toc.value == snark::imaging::colorspace::none ) { COMMA_THROW( comma::exception, "must provide destination colorspace using '--to'" ); }
            if ( options.exists( "--output-fields" ) ) { std::cout << comma::join( snark::imaging::colorspace::field_names( toc.value ), ',' ) << std::endl; return 0; }
            snark::imaging::range tor = snark::imaging::stringify::to( tov.size() > 1 ? tov[1] : snark::imaging::colorspace::default_range( toc.value ) );
            // store the output format as range for convenience of parsing
            snark::imaging::range tof = tov.size() > 1 ? ( tov.size() > 2 ? snark::imaging::stringify::to( tov[2] ) : tor ) : snark::imaging::d ;

            // the actual processing is done below
            if ( verbose ) { std::cerr << name << "convert from '" << fromc << "," << snark::imaging::stringify::from( *fromr ) << "'"
                                               <<          " to '" << toc << "," << snark::imaging::stringify::from( tor ) << "," << snark::imaging::stringify::from( tof ) << "'"
                                               << " using fields '" << comma::join( fields, ',' ) << "'" << std::endl; }
#if 0
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
#endif
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
