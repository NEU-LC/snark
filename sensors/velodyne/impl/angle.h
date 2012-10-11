#ifndef SNARK_SENSORS_IMPL_ANGLE_H_
#define SNARK_SENSORS_IMPL_ANGLE_H_

namespace snark {  namespace velodyne { namespace impl {

struct angle
{
    static double sin( unsigned int angle );
    static double cos( unsigned int angle );
};

} } } // namespace snark {  namespace velodyne { namespace impl {

#endif // #ifndef SNARK_SENSORS_IMPL_ANGLE_H_
