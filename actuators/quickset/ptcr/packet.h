#ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PACKET_H_
#define SNARK_ACTUATORS_QUICKSET_ptcr_PACKET_H_

#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/struct.h>

namespace snark { namespace quickset { namespace ptcr {

struct constants
{
    static const unsigned char stx = 0x02;
    static const unsigned char etx = 0x03;
    static const unsigned char ack = 0x06;
    static const unsigned char nak = 0x15;
    static const unsigned char esc = 0x1B;
};
  
template < typename B >
struct header : public comma::packed::packed_struct< header< B >, 3 >
{
    comma::packed::byte type;
    comma::packed::byte address;
    comma::packed::const_byte< B::id > id;
    
    header( unsigned char t ) { type = t; address = 0; }
};

template < typename B >
struct lrc : public comma::packed::packed_struct< lrc< B >, 1 >
{
    comma::packed::byte value;
    
    bool ok() const;
    void set();
};

template < typename B >
struct footer : public comma::packed::packed_struct< footer< B >, 2 >
{
    lrc< B > footer_lrc;
    comma::packed::const_byte< 0x03 > etx;
};

template < typename B >
struct packet : public comma::packed::packed_struct< packet< B >, 3 + 2 + B::size >
{
    header< B > packet_header;
    B body;
    footer< B > packet_footer;

    packet() : packet_header( constants::stx ) {}
    packet( unsigned char type ) : packet_header( type ) {}
    packet( const B& body ) : packet_header( constants::stx ), body( body ) { packet_footer.footer_lrc.set(); }
    packet( const B& body, unsigned char type ) : packet_header( type ), body( body ) { packet_footer.footer_lrc.set(); }
};

inline static unsigned char get_lrc( const char* begin, const char* end )
{
    unsigned char c = 0;
    for( const char* p = begin; p < end; ++p ) { c ^= static_cast< unsigned char >( *p ); }
    return c;
}

template < typename B >
inline void lrc< B >::set() { value = get_lrc( value.data() - B::size - 1, value.data() ); }

template < typename B >
inline bool lrc< B >::ok() const
{
    return value() == get_lrc( value.data() - B::size - 1, value.data() );
}

} } } // namespace snark { namespace quickset { namespace ptcr {

#endif // #ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PACKET_H_
