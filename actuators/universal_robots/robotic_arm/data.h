#ifndef SNARK_ACTUATORS_UR10_DATA_H 
#define SNARK_ACTUATORS_UR10_DATA_H 
#include <comma/packed/packed.h>
#include "units.h"

namespace comma { namespace packed {

/// This structure unpacks double according to standard IEEE754 - reverses the byte order.
/// The ntohd() is from the Windows socket library, for UNIX it is not defined in the BSD sockets library because there is more than one standard for float/double between different UNIX systems.
/// In our case the standard in use is the IEEE754 [1] standard (which all Windows systems also use).
template < typename T >
class big_endian_64 : public comma::packed::field< big_endian_64< T >, T, sizeof( T ) >
{
public:
    enum { size = sizeof( T ) };
    
    //BOOST_STATIC_ASSERT( size == 8 );
    
    typedef T type;
    
    typedef comma::packed::field< big_endian_64< T >, T, size > base_type;
    
    static type default_value() { return 0; }
    
    static void pack( char* storage, type value )
    {
        ::memcpy( storage, ( void* )&value, size );
        std::reverse(&storage, &storage + size );
    }
    
    static type unpack( const char* storage )
    {
        type value;
        ::memcpy( ( void* )&value, storage, size );
        
        char& raw = reinterpret_cast<char&>(value);
        std::reverse(&raw, &raw + size );
        return value;
    }
    
    const big_endian_64< T >& operator=( const big_endian_64< T >& rhs ) { return base_type::operator=( rhs ); }
    
    const big_endian_64< T >& operator=( type rhs ) { return base_type::operator=( rhs ); }
};

typedef big_endian_64< double > big_endian_double;
typedef big_endian_64< comma::uint64 > big_endian_uint64;
typedef big_endian_64< comma::int64 > big_endian_int64;
typedef big_endian_64< comma::int64 > big_endian_int64;
typedef big_endian_64< float > big_endian_float;

} } // namespace comma { namespace packed {

namespace snark { namespace robot_arm {
    
struct cartesian {
    comma::packed::big_endian_double x;
    comma::packed::big_endian_double y;
    comma::packed::big_endian_double z;
};

static const unsigned char joints_num = 6;
typedef boost::array< comma::packed::big_endian_double, joints_num > joints_net_t;

struct joints_in_degrees {
    joints_in_degrees() {}
    joints_in_degrees( const joints_net_t& joints_ ) 
    { 
        joints[0] = static_cast< plane_angle_degrees_t >( joints_[0]() * radian );  
        joints[1] = static_cast< plane_angle_degrees_t >( joints_[1]() * radian );  
        joints[2] = static_cast< plane_angle_degrees_t >( joints_[2]() * radian );  
        joints[3] = static_cast< plane_angle_degrees_t >( joints_[3]() * radian );  
        joints[4] = static_cast< plane_angle_degrees_t >( joints_[4]() * radian );  
        joints[5] = static_cast< plane_angle_degrees_t >( joints_[5]() * radian );  
    } 
    boost::array< plane_angle_degrees_t, joints_num > joints;
};
    

struct fixed_status : public comma::packed::packed_struct< fixed_status, 812  >
{
    comma::packed::big_endian_uint32 length;
    comma::packed::big_endian_double time_since_boot;
    comma::packed::string< 240 > dummy1;
    joints_net_t positions; /// actual joint positions
    joints_net_t velocities; /// actual joint velocities
    joints_net_t currents; /// actual joint currents - Amps
    comma::packed::string< 192 > dummy2;
    cartesian  translation;       ///  coordinates
    cartesian  rotation;       ///  rotation
    comma::packed::string< 56 > dummy3;
    joints_net_t  temperatures; ///  Celcius
    comma::packed::string< 16 > dummy4;
    comma::packed::big_endian_double robot_mode;
    joints_net_t joint_modes; ///  joint modes - see documents

};

template < typename T > struct packed_buffer {
    T data;
};  


struct robot_mode : public comma::packed::packed_struct< robot_mode, 29 > {
    comma::packed::uint32 length;
    unsigned char type; // should be 0
    comma::packed::big_endian_uint64 timestamp;
    unsigned char connected;
    unsigned char enabled;
    unsigned char power_on;
    unsigned char estopped;
    unsigned char security_stopped;
    unsigned char program_running;
    unsigned char program_paused;
    unsigned char robot_mode;
    comma::packed::big_endian_double speed_fraction;
};

typedef boost::array< comma::packed::big_endian_float, joints_num > joints_net_float_t;

// struct joint_data : public comma::packed::packed_struct< joint_data, 41 > {
//     comma::packed::big_endian_double position;
//     comma::packed::big_endian_double target_position;
//     comma::packed::big_endian_double speed;
//     comma::packed::string< 8 > dummy1;
//     comma::packed::big_endian_float temperature;
//     comma::uint32 unused;
//     unsigned char mode;
// };
//     
// struct joints_data : public comma::packed::packed_struct< joints_data, 251 > {
//     comma::packed::uint32 length;
//     unsigned char type; // should be 0
//     boost::array< joint_data, joints_num > joints;
// };

} } //namespace snark { namespace robot_arm {

#endif // SNARK_ACTUATORS_UR10_DATA_H 
