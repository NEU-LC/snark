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

#ifndef COMMA_MATH_INTERVAL_H_
#define COMMA_MATH_INTERVAL_H_

#include <iostream>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include <Eigen/Core>

namespace snark { namespace math {

/// multi-dimensional interval
template < typename T, int N >
class interval
{
    public:
        typedef Eigen::Matrix< T, N, 1 > vector_type;
        /// copy constructor
        interval( const interval& rhs ) { this->operator=( rhs ); }

        /// constructor
        interval( const vector_type& min, const vector_type& max ) : interval_( get_min( min, max ), get_max( min, max ) ) { if( less( max, min ) ) { COMMA_THROW( comma::exception, "invalid interval" ); } }

        /// constructor
        interval( const vector_type& min ) : interval_( min, min ) { }

        /// return value
        const std::pair< vector_type, vector_type >& operator()() const { return interval_; }

        /// return left boundary (convenience method)
        vector_type min() const { return interval_.first; }

        /// return right boundary (convenience method)
        vector_type max() const { return interval_.second; }

        /// return true, if variable belongs to the interval
        bool contains( const vector_type& t ) const { return ( ( interval_.first.isApprox( t ) || less( interval_.first, t ) ) && ( ( interval_.second.isApprox( t ) || less( t, interval_.second ) ) ) ); }

        /// return true, if variable belongs to the interval
        bool contains( const interval& rhs ) const { return !( less( rhs().first, interval_.first ) || less( interval_.second, rhs().second ) ); }

        /// compute the hull of the interval and [x]
        interval< T, N > hull( const vector_type& x ) { return interval( get_min( interval_.first, x ), get_max( interval_.second, x ) ); }

        /// compute the hull of 2 intervals
        interval< T, N > hull( const interval& rhs ) { return interval( get_min( interval_.first, rhs.min() ), get_max( interval_.second, rhs.max() ) ); }

        /// equality
        bool operator==( const interval& rhs ) const { return interval_.first.isApprox( rhs().first ) && interval_.second.isApprox( rhs().second ); }

        /// unequality
        bool operator!=( const interval& rhs ) const { return !operator==( rhs ); }

    private:
        static bool less( const vector_type& lhs, const vector_type& rhs ) { return ( ( lhs.array() < rhs.array() ).all() ); }
        static vector_type get_min( const vector_type& lhs, const vector_type& rhs ) { return rhs.array().min( lhs.array() ); }
        static vector_type get_max( const vector_type& lhs, const vector_type& rhs ) { return rhs.array().max( lhs.array() ); }
        
        std::pair< vector_type, vector_type > interval_;
};

} } // namespace snark { namespace math {

#endif // COMMA_MATH_INTERVAL_H_
