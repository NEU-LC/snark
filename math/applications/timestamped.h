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


#ifndef SNARK_APPLICATIONS_TIMESTAMPEDPOSITION_H_
#define SNARK_APPLICATIONS_TIMESTAMPEDPOSITION_H_

#include "../../timing/time.h"
#include <comma/visiting/traits.h>

namespace snark{ namespace applications {

/// a quick helper class for simple input/output of timestamped values
template < typename T, typename TimeType = boost::posix_time::ptime >
struct timestamped
{
    TimeType t;
    T value;
};

namespace detail {

inline std::string toString( const boost::posix_time::ptime& t ) { return boost::posix_time::to_iso_string( t ); }

} // namespace detail {

} } // namespace snark{ namespace applications {

template < typename T, typename S >
inline std::ostream& operator<<( std::ostream& os, const snark::applications::timestamped< T, S >& rhs ) { os << snark::applications::detail::toString( rhs.t ) << "," << rhs.value; return os; }

namespace comma { namespace visiting {

template < typename T, typename S > struct traits< snark::applications::timestamped< T, S > >
{
    template < typename Key, class Visitor >
    static void visit( Key, const snark::applications::timestamped< T, S >& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "value", p.value );
    }
    
    template < typename Key, class Visitor >
    static void visit( Key, snark::applications::timestamped< T, S >& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "value", p.value );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_APPLICATIONS_TIMESTAMPEDPOSITION_H_*/
