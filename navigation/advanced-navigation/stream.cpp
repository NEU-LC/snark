// Copyright (c) 2017 The University of Sydney

#include "stream.h"
#include <comma/base/exception.h>

namespace snark { namespace navigation { namespace advanced_navigation {

// -------------
// serial_stream
// -------------
serial_stream::serial_stream( const std::string& name, const advanced_navigation::options& options )
    : port( service, name )
{
    port.set_option( boost::asio::serial_port_base::baud_rate( options.baud_rate ));
    port.set_option( boost::asio::serial_port_base::character_size( 8 ));
    port.set_option( boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none ));
    port.set_option( boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one ));
}

std::size_t serial_stream::read_some( char* buf, std::size_t buf_size, std::size_t read_size )
{
    boost::system::error_code ec;
    std::size_t count = port.read_some( boost::asio::buffer( buf, buf_size ), ec );
    if( ec ) { COMMA_THROW( comma::exception, ec.message() ); }
    return count;
}

std::size_t serial_stream::write( const char* buf, std::size_t to_write )
{
    return boost::asio::write( port, boost::asio::buffer( buf, to_write ));
}

comma::io::file_descriptor serial_stream::fd()
{
    return port.native_handle();
}

// -------------
// io_stream
// -------------
io_stream::io_stream( const std::string& name, const advanced_navigation::options& options )
    : is( name, comma::io::mode::binary, comma::io::mode::non_blocking )
{
//     std::cerr<<"io_stream::io_stream "<<name<<" "<<(bool)ios<<std::endl;
}

std::size_t io_stream::read_some( char* buf, std::size_t buf_size, std::size_t read_size )
{
//     std::cerr<<"io_stream::read_some "<<to_read<<" "<<(void*)buf<<std::endl;
    if( !is->good() ) { throw eois_exception( std::string( "end of file on istream ") + is.name() ); }
//     return is->readsome(buf,buf_size);
    is->read( buf, read_size );
    return is->gcount();
//     unsigned read=ios->gcount();
//     std::cerr<<"io_stream::read_some / "<<read<<std::endl;
//     return read;
}

std::size_t io_stream::write( const char* buf, std::size_t to_write )
{
    COMMA_THROW( comma::exception, "cannot write to istream");
//     ios->write(buf,to_write);
//     return ios->good()?to_write:0;
}

comma::io::file_descriptor io_stream::fd()
{
    return is.fd();
}

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
