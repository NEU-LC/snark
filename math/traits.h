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

#ifndef SNARK_MATH_TRAITS_H_
#define SNARK_MATH_TRAITS_H_

#include <boost/array.hpp>
#include <Eigen/Core>

namespace snark{ namespace math{

template < typename T >
struct traits
{
    enum { size = 1 };
    T zero() { return T( 0 ); }
    T one() { return T( 1 ); }
    T identity() { return T( 1 ); }
};

template < typename T, int Rows, int Columns >
struct traits< ::Eigen::Matrix< T, Rows, Columns > >
{
    enum { rows = Rows, columns = Columns, size = rows * columns };
    
    static const ::Eigen::Matrix< T, Rows, Columns >& zero()
    {
        static ::Eigen::Matrix< T, Rows, Columns > z = ::Eigen::Matrix< T, Rows, Columns >::Zero();
        return z;        
    }
    
    static const ::Eigen::Matrix< T, Rows, Columns >& indentity()
    {
        static ::Eigen::Matrix< T, Rows, Columns > i = ::Eigen::Matrix< T, Rows, Columns >::Identity();
        return i;
    }
};

template < typename T, std::size_t Size >
class traits< boost::array< T, Size > >
{
    public:
        enum { size = Size };

        static const boost::array< T, Size >& zero() { static boost::array< T, Size > z = zero_(); return z; }
        
    private:
        static boost::array< T, Size > zero_()
        {
            boost::array< T, Size > z;
            for( std::size_t i = 0; i < Size; ++i ) { z[i] = math::traits< T >::zero(); }
            return z;
        }
};

} } // namespace snark{ namespace math{

#endif //SNARK_MATH_TRAITS_H_
