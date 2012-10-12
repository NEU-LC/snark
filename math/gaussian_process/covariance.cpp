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
#include <Eigen/Eigen>
#include "./covariance.h"

namespace snark{ 

squared_exponential_covariance::squared_exponential_covariance( double length_scale
                                                              , double signal_variance
                                                              , double data_variance )
    : signal_variance_( signal_variance )
    , data_variance_( data_variance )
    , factor_( -1.0 / ( 2.0 * std::sqrt( length_scale ) ) )
    , self_covariance_( signal_variance + data_variance )
{
}

double squared_exponential_covariance::covariance( const Eigen::VectorXd& v, const Eigen::VectorXd& w ) const
{
    const Eigen::VectorXd& diff = v - w;
    return signal_variance_ * std::exp( factor_ * diff.dot( diff ) );
}

double squared_exponential_covariance::self_covariance() const { return self_covariance_; }

} // namespace snark{ { namespace Robotics {
