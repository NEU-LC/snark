#ifndef SNARK_COMMS_PCAN_TRANSPORT_H_
#define SNARK_COMMS_PCAN_TRANSPORT_H_

#include <boost/optional.hpp>
#include <comma/io/file_descriptor.h>
#include <snark/comms/pcan/message.h>

namespace snark { namespace pcan {

struct transport
{
    virtual ~transport() {}
    
    virtual boost::optional< message > read() = 0;
    
    virtual void write( const message& m ) = 0;
    
    virtual comma::io::file_descriptor fd() const = 0;
    
    virtual void close() = 0;
};

// todo
struct char_device
{
    boost::optional< message > read() { return boost::optional< message >(); }
    
    void write( const message& m ) {}
    
    comma::io::file_descriptor fd() const { return -1; } // todo: does pcan expose file descriptor?
    
    void close();
};

// todo
struct socket
{
    boost::optional< message > read() { return boost::optional< message >(); }
    
    void write( const message& m ) {}
    
    comma::io::file_descriptor fd() const { return -1; } // todo: does pcan expose file descriptor?
    
    void close();
};

/// @todo depending on the name, create either char device or socket
transport* make_transport( const std::string& name ) { return NULL; }
    
} } // namespace snark { namespace pcan {

#endif // SNARK_COMMS_PCAN_TRANSPORT_H_
