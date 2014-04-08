#ifndef ACFR_ROVER_UTIL_UNITS_H
#define ACFR_ROVER_UTIL_UNITS_H
#include <boost/units/quantity.hpp>
#include <boost/units/base_unit.hpp>
#include <boost/concept_check.hpp>
#include <boost/units/systems/si/time.hpp>

#include <boost/units/systems/si/current.hpp>
#include <boost/units/systems/si/electric_potential.hpp>
#include <boost/units/systems/si/power.hpp>
#include <boost/units/systems/si/temperature.hpp>
#include <boost/units/systems/temperature/celsius.hpp>
#include <boost/units/systems/si/io.hpp>

namespace snark { namespace ocean {
    
typedef boost::units::quantity< boost::units::si::electric_potential > volt_t;
typedef boost::units::quantity< boost::units::si::current > current_t;
typedef boost::units::quantity< boost::units::si::power > power_t;
typedef boost::units::quantity< boost::units::celsius::temperature > temperature_t;

const volt_t::unit_type volt = boost::units::si::volt;
const current_t::unit_type ampere = boost::units::si::ampere;
const power_t::unit_type watt = boost::units::si::watt;
const temperature_t::unit_type celcius = boost::units::celsius::degree;
    

} } // namespace snark { namespace ocean {
    

#endif // ACFR_ROVER_UNITS_H
