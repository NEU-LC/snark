// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#include <comma/base/exception.h>
#include "../../math/rotation_matrix.h"
#include "stereo.h"

namespace snark { namespace camera { namespace stereo {
    
pair::pair( const config_t& config )
    : first_( config.first )
    , second_( config.second )
{
}
        
pair::pair( const pinhole::config_t& pinhole, double baseline ) : pair( pinhole, pinhole, baseline ) {}

pair::pair( const pinhole::config_t& first, const pinhole::config_t& second, double baseline )
    : first_( first, snark::pose( Eigen::Vector3d( baseline / 2, 0, 0 ), snark::roll_pitch_yaw( 0, 0, 0 ) ) )
    , second_( first, snark::pose( Eigen::Vector3d( -baseline / 2, 0, 0 ), snark::roll_pitch_yaw( 0, 0, 0 ) ) )
{
}

static ::Eigen::Affine3d affine_( const snark::pose& pose )
{
    ::Eigen::Translation3d translation;
    translation.vector() = pose.translation;
    ::Eigen::Affine3d affine = translation * snark::rotation_matrix::rotation( pose.rotation.roll(), pose.rotation.pitch(), pose.rotation.yaw() );
    return affine;
}

static ::Eigen::Affine3d ned_affine_ = affine_( snark::pose( Eigen::Vector3d::Zero(), snark::roll_pitch_yaw( M_PI / 2, 0, M_PI / 2 ) ) );

std::pair< Eigen::Vector3d, Eigen::Vector3d > pair::to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second ) const { return to_cartesian( first, second, first_.pose, second_.pose ); }

std::pair< Eigen::Vector3d, Eigen::Vector3d > pair::to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& pose ) const  { return to_cartesian( first, second, snark::pose(), pose ); }

std::pair< Eigen::Vector3d, Eigen::Vector3d > pair::to_cartesian( const Eigen::Vector2d& first, const Eigen::Vector2d& second, const snark::pose& first_pose, const snark::pose& second_pose ) const
{
    const auto& first_affine = affine_( first_pose ); // todo! precalc affine! currently, performance sucks
    const Eigen::Vector3d& fp = first_affine * ( ned_affine_ * first_.pinhole.to_cartesian( first ) );
    const Eigen::Vector3d& fc = first_affine * Eigen::Vector3d::Zero();
    const auto& second_affine = affine_( second_pose ); // todo! precalc affine! currently, performance sucks
    const Eigen::Vector3d& sp = second_affine * ( ned_affine_ * second_.pinhole.to_cartesian( second ) );
    const Eigen::Vector3d& sc = second_affine * Eigen::Vector3d::Zero();
    const Eigen::Vector3d& f = ( fp - fc ).normalized();
    const Eigen::Vector3d& s = ( sp - sc ).normalized();
    if( comma::math::equal( f.dot( s ), f.norm() * s.norm() ) ) { COMMA_THROW( comma::exception, "got collinear projection vectors on pixels: " << first.transpose() << " and " << second.transpose() ); }
    const Eigen::Vector3d& m = s.cross( f ).normalized();
    const Eigen::Vector3d& n = s.cross( m ).normalized();
    const Eigen::Vector3d& d = sp - fp;
    const Eigen::Vector3d& a = fp + f * n.dot( d ) / n.dot( f );
    const Eigen::Vector3d& b = a + m * m.dot( d );
    
//     std::cerr << "========================================================================================" << std::endl;
//     std::cerr << "                                                    first: " << first.transpose() << std::endl;
//     std::cerr << "                                               first pose: " << first_pose.translation.transpose() << " " << first_pose.rotation.roll() << " " << first_pose.rotation.pitch() << " " << first_pose.rotation.yaw() << std::endl;
//     std::cerr << "                     first_.pinhole.to_cartesian( first ): " << first_.pinhole.to_cartesian( first ).transpose() << std::endl;
//     std::cerr << "       ned_affine_ * first_.pinhole.to_cartesian( first ): " << ( ned_affine_ * first_.pinhole.to_cartesian( first ) ).transpose() << std::endl;
//     std::cerr << "                                                       fp: " << fp.transpose() << std::endl;
//     std::cerr << "                                                       fc: " << fc.transpose() << std::endl;
//     std::cerr << "                                                        a: " << a.transpose() << std::endl;
//     std::cerr << "========================================================================================" << std::endl;
//     std::cerr << "                                                   second: " << second.transpose() << std::endl;
//     std::cerr << "                                              second pose: " << second_pose.translation.transpose() << " " << second_pose.rotation.roll() << " " << second_pose.rotation.pitch() << " " << second_pose.rotation.yaw() << std::endl;
//     std::cerr << "                   second_.pinhole.to_cartesian( second ): " << second_.pinhole.to_cartesian( second ).transpose() << std::endl;
//     std::cerr << "     ned_affine_ * second_.pinhole.to_cartesian( second ): " << ( ned_affine_ * second_.pinhole.to_cartesian( second ) ).transpose() << std::endl;
//     std::cerr << "                                                       sp: " << sp.transpose() << std::endl;
//     std::cerr << "                                                       sc: " << sc.transpose() << std::endl;
//     std::cerr << "                                                        b: " << b.transpose() << std::endl;
//     std::cerr << std::endl;
    
    return std::make_pair( a, b );
}

} } } // namespace snark { namespace camera { namespace stereo {
