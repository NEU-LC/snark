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


#include <cassert>
#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "focus.h"

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
