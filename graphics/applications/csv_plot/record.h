// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>
#include <comma/base/types.h>

namespace snark { namespace graphics { namespace plotting {

struct point
{
    boost::optional< double > x;
    boost::optional< double > y;
    boost::optional< double > z;
};

struct record: public plotting::point
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    std::vector< plotting::point > series;
    
    record( unsigned int s = 0 ): block( 0 ), series( s ) {}
    static record sample( const std::string& fields, unsigned int size = 0 );
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
