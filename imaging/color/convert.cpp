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


    // TODO:
    // outr can be duplicate here to the explicit convert call in convert (or the other way around)
    typedef std::pair< colorspace::cspace, range > half_key_t;
    typedef std::pair< half_key_t, half_key_t > conversion_key_t;
    typedef std::map< conversion_key_t, Eigen::Matrix3d > conversion_map_t;

    const conversion_map_t & populate()
    {
        static conversion_map_t m;
        m[ std::make_pair( std::make_pair( colorspace::rgb, ub ), std::make_pair( colorspace::rgb, ub ) ) ] << 1, 0, 0,
                                                                                                               0, 1, 0,
                                                                                                               0, 0, 1;
        return m;
    }

    typedef std::function< void ( const comma::csv::options & csv, const Eigen::Matrix3d ) > F;

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, typename outt >
    void convert( const comma::csv::options & csv, const Eigen::Matrix3d & m )
    {
        // std::cerr << "inc,inr,outc,outr,is_int,size: " << inc << ',' << inr << ',' << outc << ',' << outr << ',' << std::is_integral< outt >::value << ',' << sizeof(outt) << std::endl;
        comma::csv::input_stream< pixel< double, inr > > is( std::cin, csv );
        comma::csv::options output_csv;
        output_csv.flush = csv.flush;
        if( csv.binary() ) { output_csv.format( comma::csv::format::value< pixel< outt, outr > >() ); }
        comma::csv::output_stream< pixel< outt, outr > > os( std::cout, output_csv );
        comma::csv::tied< pixel< double, inr >, pixel< outt, outr > > tied( is, os );
        while( is.ready() || std::cin.good() )
        {
            const pixel< double, inr > * p = is.read();
            if( !p ) { break; }
            pixel< double, outr > op( m(0,0) * p->channel0 + m(0,1) * p->channel1 + m(0,2) * p->channel2
                                    , m(1,0) * p->channel0 + m(1,1) * p->channel1 + m(1,2) * p->channel2
                                    , m(2,0) * p->channel0 + m(2,1) * p->channel1 + m(2,2) * p->channel2 );
            tied.append( pixel< outt, outr >::convert( op ) );
            if ( output_csv.flush ) { std::cout.flush(); }
        }
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    F resolve_( typename std::enable_if< std::is_integral< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
              , typename std::enable_if< std::is_integral< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        // std::cerr << "resolve_(int, int): " << outr_v << ',' << outt_v << std::endl;
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
            // outr integer, outt integer and outr > outt
            // outr float, outt integer
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
        static const conversion_map_t & m = populate();
        const conversion_key_t & key = std::make_pair( std::make_pair( inc.value, inr ), std::make_pair( outc.value, outr ) );
        conversion_map_t::const_iterator i = m.find( key );
        if ( i == m.end() ) { COMMA_THROW( comma::exception, "conversion from colorspace " << inc << ", range " << stringify::from( inr ) << " to colorspace " << outc << ", range " << stringify::from( outr ) << " is not known" ); }

        switch ( inc.value )
        {
            case colorspace::rgb:   return std::bind( inc_< colorspace::rgb   >::dispatch( inr, outc, outr, outt ), std::placeholders::_1, i->second ); break;
            case colorspace::ypbpr: return std::bind( inc_< colorspace::ypbpr >::dispatch( inr, outc, outr, outt ), std::placeholders::_1, i->second ); break;
            case colorspace::ycbcr: return std::bind( inc_< colorspace::ycbcr >::dispatch( inr, outc, outr, outt ), std::placeholders::_1, i->second ); break;
            default:
                COMMA_THROW( comma::exception, "unknown colorspace from '" << std::string( inc ) << "'" );
        }
    }

} } // namespace snark { namespace imaging {
