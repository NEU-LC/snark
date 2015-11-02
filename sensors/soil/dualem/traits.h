#pragma once
#include "message.h"
#include <comma/visiting/traits.h>

namespace comma { namespace visiting {

template < > struct traits< snark::dualem::message::pdlmn::data >
{
    template< typename K, typename V > static void visit( const K& k, const snark::dualem::message::pdlmn::data& t, V& v )
    {
        v.apply( "header", t.header );
        v.apply( "time", t.time );
        v.apply( "hcp_conductivity", t.hcp_conductivity );
        v.apply( "hcp_inphase", t.hcp_inphase );
        v.apply( "prp_conductivity", t.prp_conductivity );
        v.apply( "prp_inphase", t.prp_inphase );
    }
};

template < > struct traits< snark::dualem::message::pdlma::data >
{
    template< typename K, typename V > static void visit( const K& k, const snark::dualem::message::pdlma::data& t, V& v )
    {
        v.apply( "header", t.header );
        v.apply( "voltage", t.voltage );
        v.apply( "termprature", t.termprature );
        v.apply( "pitch", t.pitch );
        v.apply( "roll", t.roll );
    }
};

} } //namespace comma { namespace visiting {
    
