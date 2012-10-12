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

#include <cassert>
#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "./focus.h"

namespace snark {  namespace velodyne { namespace thin {

focus::focus( double rate, double ratio ) // todo: unit test
    : m_rate( rate )
    , m_ratio( ratio )
    , m_rate_in_focus( 0 )
    , m_rate_out_of_focus( rate )
{
}

double focus::rate_in_focus() const { return m_rate_in_focus; }

double focus::rate_out_of_focus() const { return m_rate_out_of_focus; }

double focus::coverage() const
{
    double c = 0;
    for( Map::const_iterator it = m_regions.begin(); it != m_regions.end(); ++it ) { c += it->second->coverage(); }
    return c;
}

void focus::update()
{
    double c = coverage();
    m_rate_in_focus = m_rate * m_ratio / c;
    if( comma::math::less( 1.0, m_rate_in_focus ) ) { m_rate_in_focus = 1.0; }
    m_rate_out_of_focus = comma::math::equal( m_ratio, 1.0 ) ? 0.0 : m_rate * ( 1 - m_ratio ) / ( 1 - c );
}

void focus::insert( std::size_t id, region* r )
{
    m_regions[id] = boost::shared_ptr< region >( r );
    update();
}

void focus::erase( std::size_t id )
{
    m_regions.erase( id );
    update();
}

} } } // namespace snark {  namespace velodyne { namespace thin {
