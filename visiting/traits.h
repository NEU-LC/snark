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

#ifndef SNARK_VISITING_TRAITS_H_
#define SNARK_VISITING_TRAITS_H_

#include <snark/math/range_bearing_elevation.h>
#include <comma/visiting/apply.h>
#include <comma/visiting/visit.h>

namespace comma { namespace visiting {

/// visiting traits
template <>
struct traits< snark::range_bearing_elevation >
{
    /// const visiting
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::range_bearing_elevation& p, Visitor& v )
    {
        v.apply( "range", p.range() );
        v.apply( "bearing", p.bearing() );
        v.apply( "elevation", p.elevation() );
    }

    /// visiting
    template < typename Key, class Visitor >
    static void visit( Key, snark::range_bearing_elevation& p, Visitor& v )
    {
        double r;
        double b;
        double e;
        v.apply( "range", r );
        v.apply( "bearing", b );
        v.apply( "elevation", e );
        p = snark::range_bearing_elevation( r, b, e );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_VISITING_TRAITS_H_

