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

#ifndef SNARK_FILTER_CONSTANT_POSITION_H
#define SNARK_FILTER_CONSTANT_POSITION_H

#include <Eigen/Core>

namespace snark{ 
namespace constant_position {
    
struct state
{
    state(): covariance( Eigen::Matrix< double, dimension, dimension >::Identity() ) {}

    void set_innovation( const Eigen::Vector3d& innovation )
    {
        position += innovation;
    }

    static const unsigned int dimension = 3u;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
};

/// constant position motion model
struct model
{
    Eigen::Matrix< double, state::dimension, 1 > sigma; /// noise sigma
    Eigen::Matrix< double, state::dimension, state::dimension > m_jacobian;
    Eigen::Matrix< double, state::dimension, state::dimension > noise;

    model(void): sigma( Eigen::Matrix< double, state::dimension, 1 >::Ones() ),
                 m_jacobian( Eigen::Matrix< double, state::dimension, state::dimension >::Identity() ),
                 noise( Eigen::Matrix< double, state::dimension, state::dimension >::Zero() )
    { }

    Eigen::Matrix< double, state::dimension, state::dimension > & jacobian( const state & state, double dt )
    {
        return m_jacobian;
    }

    void update_state( state & state, const double dt ){ }

    Eigen::Matrix< double, state::dimension, state::dimension >& noise_covariance( double dt )
    {
        for( unsigned int i = 0; i < 3; i++ )
        {
            noise(i,i) = dt * sigma[i];
        }
        return noise;
    }

};

struct position
{
    static const int dimension = 3;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
    Eigen::Matrix3d jacobian;

    position(): position( Eigen::Vector3d::Zero() ),
                covariance( Eigen::Matrix3d::Identity() ),
                jacobian( Eigen::Matrix3d::Identity() )
    {
    }

    const Eigen::Matrix3d& measurement_jacobian( const state & state ) const
    {
        return jacobian;
    }

    const Eigen::Matrix3d measurement_covariance( const state & state ) const
    {
        return covariance;
    }

    const Eigen::Matrix< double, dimension, 1 > innovation( const state & state ) const
    {
        return position;
    }
};


} } }

#endif // SNARK_FILTER_CONSTANT_POSITION_H
