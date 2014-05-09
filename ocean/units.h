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

const voltage_t::unit_type volt = boost::units::si::volt;
const current_t::unit_type ampere = boost::units::si::ampere;
const power_t::unit_type watt = boost::units::si::watt;
//const boost::units::celsius::degree_instance_t<>&  celcius = boost::units::celsius::degree;
const temperature_t::unit_type kelvin = boost::units::si::kelvin;
    
} } // namespace snark { namespace ocean {
    

#endif // ACFR_ROVER_UNITS_H
