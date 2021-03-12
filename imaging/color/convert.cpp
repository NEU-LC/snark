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

#include "convert.h"
#include "traits.h"

#include <comma/csv/stream.h>
#include <Eigen/Dense>

namespace {

    using namespace snark::imaging;

    typedef std::vector< double > channels;

    // conversion does not have to be a linear operation, but many are; these function simplify setting up such conversions
    /*
      commented-out to remove a compiler warning (-Wunused-function)
    channels linear_combination( const channels & i, const Eigen::Vector3d & before, const Eigen::Matrix3d & m, const Eigen::Vector3d & after )
    {
        channels t = { i[0] + before(0), i[1] + before(1), i[2] + before(2) };
        return channels( { m(0,0) * t[0] + m(0,1) * t[1] + m(0,2) * t[2] + after(0)
                         , m(1,0) * t[0] + m(1,1) * t[1] + m(1,2) * t[2] + after(1)
                         , m(2,0) * t[0] + m(2,1) * t[1] + m(2,2) * t[2] + after(2) } );
    }

    channels linear_combination( const channels & i, const Eigen::Vector3d & before, const Eigen::Matrix3d & m )
    {
        channels t = { i[0] + before(0), i[1] + before(1), i[2] + before(2) };
        return channels( { m(0,0) * t[0] + m(0,1) * t[1] + m(0,2) * t[2]
                         , m(1,0) * t[0] + m(1,1) * t[1] + m(1,2) * t[2]
                         , m(2,0) * t[0] + m(2,1) * t[1] + m(2,2) * t[2] } );
    }
    */
    channels linear_combination( const channels & i, const Eigen::Matrix3d & m, const Eigen::Vector3d & after )
    {
        return channels( { m(0,0) * i[0] + m(0,1) * i[1] + m(0,2) * i[2] + after(0)
                         , m(1,0) * i[0] + m(1,1) * i[1] + m(1,2) * i[2] + after(1)
                         , m(2,0) * i[0] + m(2,1) * i[1] + m(2,2) * i[2] + after(2) } );
    }

    channels linear_combination( const channels & i, const Eigen::Matrix3d & m )
    {
        return channels( { m(0,0) * i[0] + m(0,1) * i[1] + m(0,2) * i[2]
                         , m(1,0) * i[0] + m(1,1) * i[1] + m(1,2) * i[2]
                         , m(2,0) * i[0] + m(2,1) * i[1] + m(2,2) * i[2] } );
    }

    typedef std::function< channels ( const channels & p ) > C;
    // TODO:
    // - outr can be duplicate to the explicit convert call in convert (or the other way around); sort out
    typedef std::pair< colorspace::cspace, range > half_key_t;
    typedef std::pair< half_key_t, half_key_t > conversion_key_t;
    typedef std::map< conversion_key_t, C > conversion_map_t;

    C scale( range inr, range outr )
    {
        double uin, lin, uout, lout;
        switch ( inr ) {
            case ub : uin = limits< ub >::upper(); lin = limits< ub >::lower(); break;
            case uw : uin = limits< uw >::upper(); lin = limits< uw >::lower(); break;
            case ui : uin = limits< ui >::upper(); lin = limits< ui >::lower(); break;
            case f  : uin = limits< f  >::upper(); lin = limits< f  >::lower(); break;
            case d  : uin = limits< d  >::upper(); lin = limits< d  >::lower(); break;
            default:
                COMMA_THROW( comma::exception, "unknown input range '" << stringify::from( inr ) << "'" );
        }

        switch ( outr ) {
            case ub : uout = limits< ub >::upper(); lout = limits< ub >::lower(); break;
            case uw : uout = limits< uw >::upper(); lout = limits< uw >::lower(); break;
            case ui : uout = limits< ui >::upper(); lout = limits< ui >::lower(); break;
            case f  : uout = limits< f  >::upper(); lout = limits< f  >::lower(); break;
            case d  : uout = limits< d  >::upper(); lout = limits< d  >::lower(); break;
            default:
                COMMA_THROW( comma::exception, "unknown output range '" << stringify::from( outr ) << "'" );
        }

        double factor = ( uout - lout ) / ( uin - lin );
        return [ factor, lin, lout ]( const channels & rhs ){
            return channels( { factor * ( rhs[0] - lin ) + lout
                             , factor * ( rhs[1] - lin ) + lout
                             , factor * ( rhs[2] - lin ) + lout } ); };
    }

    const conversion_map_t & conversions()
    {
        static conversion_map_t m;
        m[ std::make_pair( std::make_pair( colorspace::rgb, ub ), std::make_pair( colorspace::rgb,   ub ) ) ] =
            []( const channels & i ){ return linear_combination( i, (Eigen::Matrix3d() << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished() ); };
        m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ypbpr, f  ) ) ] =
            []( const channels & i ){ return linear_combination( i, (Eigen::Matrix3d() << 0.299, 0.587, 0.114, -0.168736, -0.331264, 0.5, 0.5, -0.418688, -0.081312).finished(), (Eigen::Vector3d() << 0, 0.5, 0.5).finished() ); };
        m[ std::make_pair( std::make_pair( colorspace::rgb, d  ), std::make_pair( colorspace::ypbpr, d  ) ) ] = m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ypbpr, f  ) ) ];
        m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ypbpr, d  ) ) ] = m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ypbpr, f  ) ) ];
        m[ std::make_pair( std::make_pair( colorspace::rgb, d  ), std::make_pair( colorspace::ypbpr, f  ) ) ] = m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ypbpr, f  ) ) ];
        m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ycbcr, ub ) ) ] =
            []( const channels & i ){ return linear_combination( i, (Eigen::Matrix3d() << 65.481, 128.553, 24.966, -37.797, -74.203, 112.0, 112.0, -93.786, -18.214).finished(), (Eigen::Vector3d() << 16, 128, 128).finished() ); };
        m[ std::make_pair( std::make_pair( colorspace::rgb, d  ), std::make_pair( colorspace::ycbcr, ub ) ) ] = m[ std::make_pair( std::make_pair( colorspace::rgb, f  ), std::make_pair( colorspace::ycbcr, ub ) ) ];
        // this differs from commonly presented values, e.g., wiki@YCbCr page, that rounds numbers to 3 decimal digits
        m[ std::make_pair( std::make_pair( colorspace::rgb, ub ), std::make_pair( colorspace::ycbcr, ub ) ) ] =
            []( const channels & i ){ return linear_combination( i, (Eigen::Matrix3d() << 65.481, 128.553, 24.966, -37.797, -74.203, 112.0, 112.0, -93.786, -18.214).finished() / 255., (Eigen::Vector3d() << 16, 128, 128).finished() ); };
        {
            m[ std::make_pair( std::make_pair( colorspace::ycbcr, ub ), std::make_pair( colorspace::rgb, ub ) ) ] =
                []( const channels & i) {
                    return channels( { 255/219. * ( i[0] - 16 )                                               + 255/112.*0.701             * ( i[2] - 128 )
                                     , 255/219. * ( i[0] - 16 ) - 255/112.*0.886*0.114/0.587 * ( i[1] - 128 ) - 255/112.*0.701*0.299/0.587 * ( i[2] - 128 )
                                     , 255/219. * ( i[0] - 16 ) + 255/112.*0.886             * ( i[1] - 128 )                                               } );
                };
        }
        return m;
    }

    C select( const conversion_map_t & m, const colorspace & inc, range inr, const colorspace & outc, range outr, bool recurse = true )
    {
        // the logic for searching for the conversion method:
        // 0. check for non-conversions: same colorspace, different (or even same) range; handle explicitly
        if ( inc.value == outc.value )
        {
            if ( inr == outr ) { return []( const channels & c ) { return c; }; }
            return scale( inr, outr );
        }
        // 1. if there is a direct conversion from (inc, inr) to (outc, outr) use it (data are stored as doubles on both in and out)
        {
            const conversion_key_t & key = std::make_pair( std::make_pair( inc.value, inr ), std::make_pair( outc.value, outr ) );
            conversion_map_t::const_iterator i = m.find( key );
            if ( i != m.end() ) { return i->second; }
        }
        // 2. if there is a conversion from (inc, inr) to (outc, default-range-of-outc) use it, then change range of outc (chain conversions)
        {
            range outdef = colorspace::default_range( outc.value );
            const conversion_key_t & key = std::make_pair( std::make_pair( inc.value, inr ), std::make_pair( outc.value, outdef ) );
            conversion_map_t::const_iterator i = m.find( key );
            if ( i != m.end() ) { return [ i, outdef, outr ]( const channels & c ){ return scale( outdef, outr )( i->second( c ) ); }; }
        }
        // 3. if there is a conversion from (inc, default-range-of-inc) to (outc, outr), use it after changing range of inc
        {
            range indef = colorspace::default_range( inc.value );
            const conversion_key_t & key = std::make_pair( std::make_pair( inc.value, indef ), std::make_pair( outc.value, outr ) );
            conversion_map_t::const_iterator i = m.find( key );
            if ( i != m.end() ) { return [ i, inr, indef ]( const channels & c ){ return i->second( scale( inr, indef )( c ) ); }; }
        }
        // 4. if there is a conversion from (inc, default-range-of-inc) to (outc, default-range-of-outc), change range of inc, apply conversion, then change range of outc
        {
            range indef = colorspace::default_range( inc.value );
            range outdef = colorspace::default_range( outc.value );
            const conversion_key_t & key = std::make_pair( std::make_pair( inc.value, indef ), std::make_pair( outc.value, outdef ) );
            conversion_map_t::const_iterator i = m.find( key );
            if ( i != m.end() ) { return [ i, inr, indef, outdef, outr ]( const channels & c ){ return scale( outdef, outr )( i->second( scale( inr, indef )( c ) ) ); }; }
        }
        // 5. if there is a conversion from (inc, inr ) to (rgb, some-range-of-rgb) and also conversion from (rgb, some-range-of-rgb) to (outc, outr)
        //    ( covering all possible intermediate conversions through a recursive call), convert through the intermediate rgb
        {
            if ( recurse )
            {
                for ( range r : { ub, f } )
                {
                    try {
                        // do not stuck in infinite recursion
                        C c0 = select( m, inc, inr, colorspace::rgb, r, false );
                        C c1 = select( m, colorspace::rgb, r, outc, outr, false );
                        return [ c0, c1 ]( const channels & c ){ return c1( c0( c ) ); };
                    }
                    catch ( comma::exception & )
                    {}
                }
            }
        }
        // 6. if all the above fails, throw
        COMMA_THROW( comma::exception, "conversion from colorspace " << inc << ", range " << stringify::from( inr ) << " to colorspace " << outc << ", range " << stringify::from( outr ) << " is not known" );
    }

    typedef std::function< void ( const comma::csv::options & csv, const C & c ) > F;

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, typename outt >
    void convert( const comma::csv::options & csv, const C & c )
    {
        pixel< double, inr > sample_in;
        comma::csv::input_stream< pixel< double, inr > > is( std::cin, csv, sample_in );
        comma::csv::options output_csv;
        output_csv.full_xpath = false;
        output_csv.flush = csv.flush;
        if( csv.binary() ) { output_csv.format( comma::csv::format::value< pixel< outt, outr > >() ); }
        pixel< outt, outr > sample_out;
        comma::csv::output_stream< pixel< outt, outr > > os( std::cout, output_csv, sample_out );
        comma::csv::tied< pixel< double, inr >, pixel< outt, outr > > tied( is, os );
        pixel< double, outr > op;
        while( is.ready() || std::cin.good() )
        {
            const pixel< double, inr > * ip = is.read();
            if( !ip ) { break; }
            c( ip->channel ).swap( op.channel );
            tied.append( pixel< outt, outr >( op ) );
            if ( output_csv.flush ) { std::cout.flush(); }
        }
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    F resolve_( typename std::enable_if< std::is_integral< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
              , typename std::enable_if< std::is_integral< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        if ( outr_v == ub && outt_v == ub ) { return convert< inc, inr, outc, ub, typename range_traits< ub >::value_t >; }
        if ( outr_v == ub && outt_v == uw ) { return convert< inc, inr, outc, ub, typename range_traits< uw >::value_t >; }
        if ( outr_v == ub && outt_v == ui ) { return convert< inc, inr, outc, ub, typename range_traits< ui >::value_t >; }
        if ( outr_v == uw && outt_v == uw ) { return convert< inc, inr, outc, uw, typename range_traits< uw >::value_t >; }
        if ( outr_v == uw && outt_v == ui ) { return convert< inc, inr, outc, uw, typename range_traits< ui >::value_t >; }
        if ( outr_v == ui && outt_v == ui ) { return convert< inc, inr, outc, ui, typename range_traits< ui >::value_t >; }
        COMMA_THROW( comma::exception, "unsupported combination of output range " << stringify::from( outr ) << " and type " << stringify::from( outt ) );
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    F resolve_( typename std::enable_if< std::is_floating_point< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
              , typename std::enable_if< std::is_integral< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        COMMA_THROW( comma::exception, "cannot use integer output type " << stringify::from( outt ) << " for " << stringify::from( outr ) << " output range" );
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    F resolve_( typename std::enable_if< std::is_integral< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
              , typename std::enable_if< std::is_floating_point< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        return convert< inc, inr, outc, outr, typename range_traits< outt >::value_t >;
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    F resolve_( typename std::enable_if< std::is_floating_point< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
              , typename std::enable_if< std::is_floating_point< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        return convert< inc, inr, outc, outr, typename range_traits< outt >::value_t >;
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    struct outt_
    {
        static F dispatch()
        {
            // shall not attempt instantiating if:
            // - outr is integer, outt is integer and outr > outt
            // - outr is float, outt is integer
            typename range_traits< outr >::value_t outr_v( outr );
            typename range_traits< outt >::value_t outt_v( outt );
            return resolve_< inc, inr, outc, outr, outt >( outr_v, outt_v );
        }
    };

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr >
    struct outr_
    {
        static F dispatch( range outt )
        {
            switch ( outt )
            {
                case ub: return outt_< inc, inr, outc, outr, ub >::dispatch( ); break;
                case uw: return outt_< inc, inr, outc, outr, uw >::dispatch( ); break;
                case ui: return outt_< inc, inr, outc, outr, ui >::dispatch( ); break;
                case f:  return outt_< inc, inr, outc, outr, f  >::dispatch( ); break;
                case d:  return outt_< inc, inr, outc, outr, d  >::dispatch( ); break;
                default:
                    COMMA_THROW( comma::exception, "unknown output storage format '" << stringify::from( outt ) << "'" );
            }
        }
    };

    template< colorspace::cspace inc, range inr, colorspace::cspace outc >
    struct outc_
    {
        static F dispatch( range outr, range outt )
        {
            switch ( outr )
            {
                case ub: return outr_< inc, inr, outc, ub >::dispatch( outt ); break;
                case uw: return outr_< inc, inr, outc, uw >::dispatch( outt ); break;
                case ui: return outr_< inc, inr, outc, ui >::dispatch( outt ); break;
                case f:  return outr_< inc, inr, outc, f  >::dispatch( outt ); break;
                case d:  return outr_< inc, inr, outc, d  >::dispatch( outt ); break;
                default:
                    COMMA_THROW( comma::exception, "unknown output range '" << stringify::from( outr ) << "'" );
            }
        }
    };

    template< colorspace::cspace inc, range inr >
    struct inr_
    {
        static F dispatch( const colorspace & outc, range outr, range outt )
        {
            switch ( outc.value )
            {
                case colorspace::rgb:   return outc_< inc, inr, colorspace::rgb   >::dispatch( outr, outt ); break;
                case colorspace::ypbpr: return outc_< inc, inr, colorspace::ypbpr >::dispatch( outr, outt ); break;
                case colorspace::ycbcr: return outc_< inc, inr, colorspace::ycbcr >::dispatch( outr, outt ); break;
                default:
                    COMMA_THROW( comma::exception, "unknown colorspace to '" << std::string( outc ) << "'" );
            }
        }
    };

    template< colorspace::cspace inc >
    struct inc_
    {
        static F dispatch( range inr, const colorspace & outc, range outr, range outt )
        {
            switch ( inr )
            {
                case ub: return inr_< inc, ub >::dispatch( outc, outr, outt ); break;
                case uw: return inr_< inc, uw >::dispatch( outc, outr, outt ); break;
                case ui: return inr_< inc, ui >::dispatch( outc, outr, outt ); break;
                case f:  return inr_< inc, f  >::dispatch( outc, outr, outt ); break;
                case d:  return inr_< inc, d  >::dispatch( outc, outr, outt ); break;
                default:
                    COMMA_THROW( comma::exception, "unknown input range '" << stringify::from( inr ) << "'" );
            }
        }
    };

} // anonymous

namespace snark { namespace imaging {

    converter::F converter::dispatch( const colorspace & inc, range inr, const colorspace & outc, range outr, range outt )
    {
        const C & conv = select( conversions(), inc, inr, outc, outr );

        switch ( inc.value )
        {
            case colorspace::rgb:   return std::bind( inc_< colorspace::rgb   >::dispatch( inr, outc, outr, outt ), std::placeholders::_1, conv ); break;
            case colorspace::ypbpr: return std::bind( inc_< colorspace::ypbpr >::dispatch( inr, outc, outr, outt ), std::placeholders::_1, conv ); break;
            case colorspace::ycbcr: return std::bind( inc_< colorspace::ycbcr >::dispatch( inr, outc, outr, outt ), std::placeholders::_1, conv ); break;
            default:
                COMMA_THROW( comma::exception, "unknown colorspace from '" << std::string( inc ) << "'" );
        }
    }

    void converter::list( std::ostream & os )
    {
        const auto & allc = { colorspace( colorspace::rgb ), colorspace( colorspace::ycbcr ), colorspace( colorspace::ypbpr ) };
        const auto & allr = { ub, uw, ui, f, d };
        for ( const auto & inc : allc )
        {
            for ( auto inr : allr )
            {
                for ( const auto & outc : allc )
                {
                    for ( auto outr : allr )
                    {
                        try
                        {
                            select( conversions(), inc, inr, outc, outr );
                            os << inc << ',' << stringify::from( inr ) << ',' << outc << ',' << stringify::from( outr ) << std::endl;
                        }
                        catch ( comma::exception & )
                        {}
                    }
                }
            }
        }
    }

} } // namespace snark { namespace imaging {
