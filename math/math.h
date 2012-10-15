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

#ifndef SNARK_MATH_MATH_H_
#define SNARK_MATH_MATH_H_

#include <Eigen/Core>
#include <comma/math/compare.h>

namespace comma { namespace math {

template < typename T >
inline bool less( const Eigen::Matrix< T, 3, 1 >& lhs, const Eigen::Matrix< T, 3, 1 >& rhs )
{
    return ( ( lhs.array() < rhs.array() ).all() );
}

template < typename T, int N >
inline Eigen::Matrix< T, N, 1 > min( const Eigen::Matrix< T, N, 1 >& rhs, const Eigen::Matrix< T, N, 1 >& lhs )
{
    return rhs.array().min( lhs.array() );
}

template < typename T, int N >
inline Eigen::Matrix< T, N, 1 > max( const Eigen::Matrix< T, N, 1 >& rhs, const Eigen::Matrix< T, N, 1 >& lhs )
{
    return rhs.array().max( lhs.array() );
}

} } // namespace comma { namespace math {
    
#endif //SNARK_MATH_MATH_H_
