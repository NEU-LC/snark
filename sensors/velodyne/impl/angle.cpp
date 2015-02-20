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


#include <cmath>
#include <boost/array.hpp>
#include <comma/base/exception.h>
#include "angle.h"

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
