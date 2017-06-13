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

namespace {

    using namespace snark::imaging;

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, typename outt >
    void convert( const comma::csv::options & csv )
    {
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
            // TODO: implement the correct conversion
            pixel< double, outr > op( p->channel0, p->channel1, p->channel2 );
            tied.append( pixel< outt, outr >::convert( op ) );
            if ( output_csv.flush ) { std::cout.flush(); }
        }
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    converter::F resolve_( typename std::enable_if< std::is_integral< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
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
    converter::F resolve_( typename std::enable_if< std::is_floating_point< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
                         , typename std::enable_if< std::is_integral< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        COMMA_THROW( comma::exception, "cannot use integer output type " << stringify::from( outt ) << " for " << stringify::from( outr ) << " output range" );
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    converter::F resolve_( typename std::enable_if< std::is_integral< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
                         , typename std::enable_if< std::is_floating_point< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        return convert< inc, inr, outc, outr, typename range_traits< outt >::value_t >;
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    converter::F resolve_( typename std::enable_if< std::is_floating_point< typename range_traits< outr >::value_t >::value, typename range_traits< outr >::value_t >::type outr_v
                         , typename std::enable_if< std::is_floating_point< typename range_traits< outt >::value_t >::value, typename range_traits< outt >::value_t >::type outt_v )
    {
        return convert< inc, inr, outc, outr, typename range_traits< outt >::value_t >;
    }

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr, range outt >
    struct outt_
    {
        static converter::F dispatch()
        {
            // shall not attempt instantiating if:
            // outr integer, outt integer and outr > outt
            // outr float, outt integer
            typename range_traits< outr >::value_t outr_v( 0 );
            typename range_traits< outt >::value_t outt_v( 0 );
            return resolve_< inc, inr, outc, outr, outt >( outr_v, outt_v );
        }
    };

    template< colorspace::cspace inc, range inr, colorspace::cspace outc, range outr >
    struct outr_
    {
        static converter::F dispatch( range outt )
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
        static converter::F dispatch( range outr, range outt )
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
        static converter::F dispatch( const colorspace & outc, range outr, range outt )
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
        static converter::F dispatch( range inr, const colorspace & outc, range outr, range outt )
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
        switch ( inc.value )
        {
            case colorspace::rgb:   return inc_< colorspace::rgb   >::dispatch( inr, outc, outr, outt ); break;
            case colorspace::ypbpr: return inc_< colorspace::ypbpr >::dispatch( inr, outc, outr, outt ); break;
            case colorspace::ycbcr: return inc_< colorspace::ycbcr >::dispatch( inr, outc, outr, outt ); break;
            default:
                COMMA_THROW( comma::exception, "unknown colorspace from '" << std::string( inc ) << "'" );
        }
    }

} } // namespace snark { namespace imaging {
