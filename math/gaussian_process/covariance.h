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

#ifndef SNARK_GAUSSIAN_PROCESS_COVARIANCE_
#define SNARK_GAUSSIAN_PROCESS_COVARIANCE_

#include <Eigen/Core>

namespace snark{ 

class squared_exponential_covariance
{
    public:
        squared_exponential_covariance( double length_scale
                                      , double signal_variance
                                      , double data_variance );

        double covariance( const Eigen::VectorXd& v, const Eigen::VectorXd& w ) const;
        
        double self_covariance() const;

    private:
        double signal_variance_;
        double data_variance_;
        double factor_;
        double self_covariance_;
};

} // namespace snark{

#endif // #ifndef SNARK_GAUSSIAN_PROCESS_COVARIANCE_
