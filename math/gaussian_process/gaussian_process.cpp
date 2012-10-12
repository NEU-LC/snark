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
