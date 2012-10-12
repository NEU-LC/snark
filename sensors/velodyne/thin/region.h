// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_SENSORS_VELODYNE_THIN_REGION
#define SNARK_SENSORS_VELODYNE_THIN_REGION

#include <comma/math/cyclic.h>
#include <comma/visiting/traits.h>

namespace snark {  namespace velodyne { namespace thin {

/// region, quick and dirty
struct region
{
    virtual ~region() {}
    virtual bool has( double range, double bearing, double elevation ) const = 0;
    virtual double coverage() const = 0;
};

/// sector, quick and dirty
struct sector : public region
{
    sector();
    sector( double bearing, double ken, double range = 0 );
    bool has( double range, double bearing, double ) const;
    double coverage() const;
    comma::math::cyclic< double > bearing;
    double ken;
    double range;
};

} } } // namespace snark {  namespace velodyne { namespace thin {

namespace comma { namespace visiting { // quick and dirty

template <> struct traits< snark::velodyne::thin::sector >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::velodyne::thin::sector& p, Visitor& v )
    {
        v.apply( "bearing", p.bearing() );
        v.apply( "ken", p.ken );
        v.apply( "range", p.range );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, snark::velodyne::thin::sector& p, Visitor& v )
    {
        double bearing = p.bearing();
        v.apply( "bearing", bearing );
        p.bearing = bearing;
        //p.bearing = comma::math::cyclic< double >( snark::math::Interval< double >( -180.0, 180.0 ), bearing );
        v.apply( "ken", p.ken );
        v.apply( "range", p.range );
    }
};

} } // namespace comma { namespace visiting {

#endif // #ifndev SNARK_SENSORS_VELODYNE_THIN_REGION

