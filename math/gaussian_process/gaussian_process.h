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

#ifndef SNARK_GAUSSIAN_PROCESS_
#define SNARK_GAUSSIAN_PROCESS_

#include <boost/function.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace snark{ 

/// gaussian process
class gaussian_process
{
    public:
        /// covariance functor type
        typedef boost::function< double ( const Eigen::VectorXd&, const Eigen::VectorXd& ) > covariance;

        /// constructor
        gaussian_process( const Eigen::MatrixXd& domains
                       , const Eigen::VectorXd& targets
                       , const gaussian_process::covariance& covariance
                       , double self_covariance = 0 );

        /// evaluate
        void evaluate( const Eigen::MatrixXd& domains
                     , Eigen::VectorXd& means
                     , Eigen::VectorXd& variances ) const;

        /// evaluate a single domain, return mean-variance pair
        std::pair< double, double > evaluate( const Eigen::MatrixXd& domain ) const;

    private:
        Eigen::MatrixXd domains_; //!< domain locations corresponding to targets
        Eigen::VectorXd targets_; //!< targets
        covariance covariance_;
        double self_covariance_;
        double offset_;
        Eigen::MatrixXd K_; //!< the inverse of (Kxx + noiseVariance*I)
        Eigen::LLT< Eigen::MatrixXd  > L_; //!< cholesky factorization of the covariance Kxx
        Eigen::VectorXd alpha_;
};

} // namespace snark{

#endif // #ifndef SNARK_GAUSSIAN_PROCESS_
