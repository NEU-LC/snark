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
