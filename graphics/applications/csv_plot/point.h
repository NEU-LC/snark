// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>
#include <comma/base/types.h>

namespace snark { namespace graphics { namespace plotting {

// struct point
// {
//     double x;
//     double y;
//     double z;
//     
//     point(): x( 0 ), y( 0 ), z( 0 ) {}
// };

struct point
{
    boost::optional< double > x;
    boost::optional< double > y;
    boost::optional< double > z;
    
    point() {}
};

struct record: public plotting::point
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    std::vector< plotting::point > series;
    
    record( unsigned int s = 0 ): block( 0 ), series( s ) {}
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
