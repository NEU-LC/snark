#include "message.h"
#include <iostream>

namespace snark { namespace hokuyo {
    
bool scip_verify_checksum(const std::string& line)
{
    if( line.size() < 2 ) { COMMA_THROW( comma::exception, "SCIP data and checksum must be at least 2 bytes long" ); }

    comma::uint32 sum = 0;
    for( std::size_t i=0; i<line.size()-1; ++i ) { sum += line[i]; } 
//     std::cerr  << "sum of " << line.substr( 0, line.size()-1 ) << " (" << line.size()-1 << "bytes)" << " is " << sum << std::endl;
    sum = (sum & hokuyo::mask) + 0x30;
//     std::cerr  << "after mask " << sum << std::endl;
    return ( sum == comma::uint32( line[ line.size()-1 ] ) );
}

    
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
        if( !scip_verify_checksum( std::string( raw, block-1 ) ) ) { COMMA_THROW( comma::exception, "checksum of data failed" );  }
        raw += block;
        target += data_size; // advance pointer pass data, checksum and line feed
        
        size -= block;
    }
    
    if( size > 0 )
    {
        if( !scip_verify_checksum( std::string( raw, size-1 ) ) ) { COMMA_THROW( comma::exception, "checksum of data failed." );  }
        // if size is 1, 2 or 3, then it is an error
        memcpy( target, raw, size-3 ); // it ends in triplet <sum, lf, lf>
    }
}
    
} // namespace details {

template < int STEPS >    
void distance_data< STEPS >::get_values( rays& points ) const
{
    details::strip_checksum( value, points.data(), raw_data.data() );
}

template < int STEPS >    
void di_data< STEPS >::get_values( rays& points ) const
{
    details::strip_checksum( value, points.data(), raw_data.data() );
}

// These are for unit tests, would never use 4 or 11 rays only
template class distance_data< 4 >;    
template class di_data< 4 >;    
template class distance_data< 11 >;    
template class di_data< 11 >;    
template class distance_data< 101 >;    
template class di_data< 101 >;    

template class distance_data< 1081 >;    
template class distance_data< 271 >;    
template class di_data< 1081 >;    
template class di_data< 271 >;    

} } // namespace snark { namespace hokuyo {
