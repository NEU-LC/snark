#ifndef SNARK_SENSORS_DC1394_TYPES_H_
#define SNARK_SENSORS_DC1394_TYPES_H_

#include <dc1394/dc1394.h>
#include <string>

namespace snark { namespace camera {

std::string video_mode_to_string( dc1394video_mode_t mode );
std::string operation_mode_to_string( dc1394operation_mode_t mode );
std::string iso_speed_to_string( dc1394speed_t speed );
std::string frame_rate_to_string( dc1394framerate_t frame_rate );

dc1394video_mode_t video_mode_from_string( const std::string& mode );
dc1394operation_mode_t operation_mode_from_string( const std::string& mode );
dc1394speed_t iso_speed_from_string( const std::string& speed );
dc1394framerate_t frame_rate_from_string( const std::string& frame_rate );

void print_video_modes();
void print_operation_modes();
void print_iso_speeds();
void print_frame_rates();

} } // namespace snark { namespace camera {


#endif // SNARK_SENSORS_DC1394_TYPES_H_
