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

#include <cmath>
#include <boost/array.hpp>
#include <comma/base/exception.h>
#include "./angle.h"

namespace snark {  namespace velodyne { namespace impl {

class prefilled_sin_table
{    
    public:
        prefilled_sin_table()
        {
            for( std::size_t i = 0; i < m_sin.size(); ++i ) { m_sin[i] = ::sin( M_PI / 180 * i ); }
            for( std::size_t i = 0; i < m_fractionSin.size(); ++i )
            {
                static double a( M_PI * 0.01 / 180 );
                m_fractionSin[i] = ::sin( a * i );
                m_fractionCos[i] = ::cos( a * i );
            }
        }
    
        double sin( unsigned int degrees, unsigned int fractions )
        {
            degrees %= 360;
            int sign( 1 );
            if( degrees >= 180 ) { sign = -1; degrees -= 180; }
            if( degrees >= 90 )
            {
                degrees = 180 - degrees;
                if( fractions > 0 ) { --degrees; fractions = 100 - fractions; }
            }
            int signOfSum( 1 );
            if( fractions > 50 )
            {
                signOfSum = -1;
                fractions = 100 - fractions;
                ++degrees;
            }
            return (   m_fractionCos[ fractions ] * m_sin[ degrees ]
                     + m_fractionSin[ fractions ] * m_sin[ 90 - degrees ] * signOfSum ) * sign;
        }
    
        double cos( unsigned int degrees, unsigned int fractions )
        {
            degrees %= 360;
            if( fractions == 0 ) { return prefilled_sin_table::sin( 360 + 90 - degrees, 0 ); }
            return prefilled_sin_table::sin( 360 + 90 - 1 - degrees, 100 - fractions );
        }
    
    private:
        // it could be further optimized, considering angle divided by
        // 256 instread 100 (thus saving on divisions), but it would become
        // even less readable
        boost::array< double, 91 > m_sin;
        boost::array< double, 51 > m_fractionSin;
        boost::array< double, 51 > m_fractionCos;
};

static prefilled_sin_table table;

double angle::sin( unsigned int angle )
{
    return table.sin( angle / 100, angle % 100 );
}

double angle::cos( unsigned int angle )
{
    return table.cos( angle / 100, angle % 100 );
}

} } } // namespace snark {  namespace velodyne { namespace impl {
