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

#ifndef SNARK_OCEAN_UTIL_UNITS_H
#define SNARK_OCEAN_UTIL_UNITS_H
#include <boost/units/quantity.hpp>
#include <boost/units/base_unit.hpp>
#include <boost/concept_check.hpp>
#include <boost/units/systems/si/time.hpp>

#include <boost/units/systems/si/current.hpp>
#include <boost/units/systems/si/electric_potential.hpp>
#include <boost/units/systems/si/power.hpp>
#include <boost/units/systems/si/temperature.hpp>
#include <boost/units/systems/temperature/celsius.hpp>
#include <boost/units/systems/si/temperature.hpp>
#include <boost/units/systems/si/io.hpp>

namespace snark { namespace ocean {
    
typedef boost::units::quantity< boost::units::si::electric_potential > voltage_t;
typedef boost::units::quantity< boost::units::si::current > current_t;
typedef boost::units::quantity< boost::units::si::power > power_t;
typedef boost::units::quantity< boost::units::si::temperature > temperature_t;
typedef boost::units::quantity< boost::units::celsius::temperature > celcius_t;

const voltage_t::unit_type volt = boost::units::si::volt;
const current_t::unit_type ampere = boost::units::si::ampere;
const power_t::unit_type watt = boost::units::si::watt;
const celcius_t::unit_type celcius = boost::units::celsius::degree;
const temperature_t::unit_type kelvin = boost::units::si::kelvin;
    
} } // namespace snark { namespace ocean {
    

#endif // ACFR_ROVER_UNITS_H
