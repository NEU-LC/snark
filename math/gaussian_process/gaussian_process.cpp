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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <boost/noncopyable.hpp>
#include <comma/base/exception.h>
#include <snark/math/gaussian_process/gaussian_process.h>

namespace snark{ 

gaussian_process::gaussian_process( const Eigen::MatrixXd& domains
                                , const Eigen::VectorXd& targets
                                , const gaussian_process::covariance& covariance
                                , double self_covariance )
    : domains_( domains )
    , targets_( targets )
    , covariance_( covariance )
    , self_covariance_( self_covariance )
    , offset_( targets.sum() / targets.rows() )
    , K_( domains.rows(), domains.rows() )
{
    if( domains.rows() != targets.rows() ) { COMMA_THROW( comma::exception, "expected " << domains.rows() << " row(s) in targets, got " << targets.rows() << " row(s)" ); }
    targets_.array() -= offset_; // normalise
    //use m_K as Kxx + variance*I, then invert it
    //fill Kxx with values from covariance function
    //for elements r,c in upper triangle
    for( std::size_t r = 0; r < std::size_t( domains.rows() ); ++r )
    {
        K_( r, r ) = self_covariance_;
        const Eigen::VectorXd& row = domains.row( r );
        for( std::size_t c = r + 1; c < std::size_t( domains.rows() ); ++c )
        {
            K_( c, r ) = K_( r, c ) = covariance_( row, domains.row( c ) );
        }
    }
    L_.compute( K_ ); // invert Kxx + variance * I to become (by definition) B
    alpha_ = L_.solve( targets_ );
}

std::pair< double, double > gaussian_process::evaluate( const Eigen::MatrixXd& domain ) const
{
    if( domain.rows() != 1 ) { COMMA_THROW( comma::exception, "expected 1 row in domain, got " << domain.rows() << " rows" ); }
    Eigen::VectorXd means( 1 );
    Eigen::VectorXd variances( 1 );
    evaluate( domain, means, variances );
    return std::make_pair( means( 0 ), variances( 0 ) );
}

void gaussian_process::evaluate( const Eigen::MatrixXd& domains, Eigen::VectorXd& means, Eigen::VectorXd& variances ) const
{
    if( domains.cols() != domains_.cols() ) { COMMA_THROW( comma::exception, "expected " << domains_.cols() << " column(s) in domains, got " << domains.cols() << std::endl ); }
    Eigen::MatrixXd Kxsx = Eigen::MatrixXd::Zero( domains.rows(), domains_.rows() );
    for( std::size_t r = 0; r < std::size_t( domains.rows() ); ++r )
    {
        const Eigen::VectorXd& row = domains.row( r );
        for( std::size_t c = 0; c < std::size_t( domains_.rows() ); ++c )
        {
            Kxsx( r, c ) = covariance_( row, domains_.row( c ) );
        }
    }
    means = Kxsx * alpha_;
    means.array() += offset_;
    Eigen::MatrixXd Kxxs = Kxsx.transpose();
    L_.matrixL().solveInPlace( Kxxs );
    Eigen::MatrixXd& variance = Kxxs;
    variance = variance.array() * variance.array();
    variances = variance.colwise().sum();
    // for each diagonal variance, set v(r) = -v(r,r) + Kxsxs
    for( std::size_t r = 0; r < std::size_t( domains.rows() ); ++r )
    {
        variances( r ) = -variances( r ) + self_covariance_;
    }
}

}  // namespace snark{ 
