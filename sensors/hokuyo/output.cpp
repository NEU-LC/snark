#include "output.h"
namespace snark { namespace hokuyo {


/// This is for setting acquired data for the point while keeping timestamp the same.
/// 'distance' is in mm
void data_point::set(double distance, comma::uint32 intensity, double bearing)
{
    this->range = distance / 1000.0;
    if( distance == ust_10lx::distance_nan || distance <= ust_10lx::distance_min )
    {
        // timestamp stays the same
        x = 0;
        y = 0;
        z = 0; 
        intensity = 0;
        bearing = 0;
        range = 0;
        return;
    }
    
    this->intensity = intensity;    
    this->bearing = bearing;
    // timestamp stays the same
    x = range * std::cos( bearing );
    y = range * -std::sin( bearing );
}

} }  // namespace snark { namespace hokuyo {
