#include "stdio_query.h"
#include <comma/base/exception.h>

namespace snark { namespace ocean { 


std::size_t stdio_query::write(const char* buf, std::size_t size)
{
    boost::system::error_code ec;
    std::size_t sent = output_.write_some(boost::asio::buffer( buf, size ), ec );

    if( ec.value() != boost::system::errc::success )
    {
        std::ostringstream ss;
        ss << "stdin write failed with error: " << ec.value() 
           << " category: " << ec.category().name() << " - " << ec.message() << std::endl;
        COMMA_THROW( comma::exception, ss.str() );
    }
    
    return sent;

}



stdio_query::stdio_query() 
    : error_code_( ),
      received_length_(0),
      desc_service_( io_service_ ),
      input_( io_service_, ::dup(STDIN_FILENO ) ),
      output_( io_service_, ::dup(STDOUT_FILENO ) ),
      deadline_( io_service_ )
{
    /// set deadline as infinite - no deadline
    deadline_.expires_at(boost::posix_time::pos_infin);
    
    // Start the deadline handler 
    check_deadline();
    
}

void stdio_query::receive( std::string& data, const boost::posix_time::time_duration& timeout_duration )
{
    boost::asio::mutable_buffer buf( &data[0], data.size() );
    boost::system::error_code ec;
    std::size_t s = receive( buf, timeout_duration, ec );
    
    if( ec.value() != boost::system::errc::success )
    {
        std::ostringstream ss;
        ss << "stdin read failed with error: " << ec.value() 
           << " category: " << ec.category().name() << " - " << ec.message() << std::endl;
        COMMA_THROW( comma::exception, ss.str() );
    }
    //success
    data.resize(s);
}

void stdio_query::async_receive(const boost::asio::mutable_buffer& buffer, boost::posix_time::time_duration timeout)
{
    /// deadline updated for receive
    deadline_.expires_from_now(timeout);
    
    error_code_ = boost::asio::error::would_block;
    received_length_ = 0;
    
    input_.async_read_some(boost::asio::buffer(buffer),
                          boost::bind(&stdio_query::handle_receive, _1, _2, &error_code_, &received_length_));
}


std::size_t stdio_query::receive(const boost::asio::mutable_buffer& buffer, 
                                       boost::posix_time::time_duration timeout, boost::system::error_code& ec)
{
    // Set a deadline for the asynchronous operation.
    deadline_.expires_from_now(timeout);
    
    // error of result of operation
    ec = boost::asio::error::would_block;
    std::size_t length = 0;
    
    input_.async_read_some(boost::asio::buffer(buffer),
                          boost::bind(&stdio_query::handle_receive, _1, _2, &ec, &length));
    // boost::asio::async_read( input_, boost::asio::buffer(buffer),
    //                         boost::asio::transfer_at_least(1),
    //                         boost::bind(&stdio_query::handle_receive, _1, _2, &ec, &length));
    
    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);
    
    return length;
}

} } // namespace snark { namespace ocean { 