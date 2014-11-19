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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_UNITS_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_UNITS_H

#include <boost/units/quantity.hpp>
#include <boost/units/base_unit.hpp>
#include <boost/concept_check.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <boost/units/systems/si/current.hpp>
#include <boost/units/systems/temperature/celsius.hpp>

namespace snark { namespace ur {
    typedef boost::units::quantity< boost::units::si::plane_angle > plane_angle_t;
    typedef boost::units::quantity< boost::units::degree::plane_angle > plane_angle_degrees_t;
    typedef boost::units::quantity< boost::units::si::length > length_t;
    typedef boost::units::quantity< boost::units::si::angular_acceleration > angular_acceleration_t;
    typedef boost::units::quantity< boost::units::si::angular_velocity > angular_velocity_t;
    typedef boost::units::quantity< boost::units::si::current > current_t;
    typedef boost::units::quantity< boost::units::celsius::temperature > celcius_t;

    const plane_angle_t::unit_type radian = boost::units::si::radian;
    const plane_angle_degrees_t::unit_type degree = boost::units::degree::degrees;
    const length_t::unit_type meter = boost::units::si::meter;
    const angular_acceleration_t::unit_type rad_per_s2 = angular_acceleration_t::unit_type();
    const angular_velocity_t::unit_type rad_per_sec = angular_velocity_t::unit_type();
    const current_t::unit_type ampere = boost::units::si::ampere;
    const celcius_t::unit_type celcius = boost::units::celsius::degree;
} } // namespace snark { namespace ur {

#endif //  SNARK_ACTUATORS_UR_ROBOTIC_ARM_UNITS_H
