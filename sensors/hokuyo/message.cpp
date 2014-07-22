#include "message.h"

namespace snark { namespace hokuyo {
    
// comma::uint32 request_gd::sequence = 0;
    
namespace details {

void strip_checksum( std::size_t raw_size, char* target, const char* raw )
{
    comma::uint32 size = raw_size;
    const char block = 66; // every block ( 64 bytes ) ending in checksum bit and line feed byte
    const char data_size = 64;
    
    // strip out all sum + line feed
    while( size >= block )
    {
        memcpy( target, raw, data_size );
        
        if( raw[block-1] != '\n' ) { COMMA_THROW( comma::exception, "failed to find line feed after 64 data bytes and checksum byte" ); }
        // verify checksum
        char sum = raw[block-2];
        comma::uint32 calulated_sum = 0;
        for( std::size_t i=0; i<data_size; ++i ) { calulated_sum += int(raw[i]); }
        // checksum is just 6 bits of calculated sum
        if( ( calulated_sum & hokuyo::mask ) != comma::uint32(sum) ) { COMMA_THROW( comma::exception, "checksum of data failed" );  }
        
        raw += block;
        target += data_size; // advance pointer pass data, checksum and line feed
    }
    
    if( size > 0 )
    {
        // if size is 1, 2 or 3, then it is an error
        memcpy( target, raw, size-3 ); // it ends in triplet <sum, lf, lf>
    }
}
    
} // namespace details {

template < int STEPS >    
void distance_data< STEPS >::get_values( rays& points )
{
    details::strip_checksum( value, raw_data.data(), points.data() );
}

template < int STEPS >    
void di_data< STEPS >::get_values( rays& points )
{
    details::strip_checksum( value, raw_data.data(), points.data() );
}

template class distance_data< 4 >;    
template class di_data< 4 >;    
template class distance_data< 1080 >;    
template class di_data< 1080 >;    

} } // namespace snark { namespace hokuyo {
