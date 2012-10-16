#ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PROTOCOL_H_
#define SNARK_ACTUATORS_QUICKSET_ptcr_PROTOCOL_H_

#include <string>
#include <boost/noncopyable.hpp>
#include "./commands.h"
#include "./packet.h"

namespace snark { namespace quickset { namespace ptcr {

class protocol : public boost::noncopyable
{
    public:
        protocol( const std::string& name );

        ~protocol();
        
        template < typename C >
        const packet< typename C::response >* send( const C& command );

        void close();
        
    private:
        class impl;
        impl* pimpl_;
};

} } } // namespace snark { namespace quickset { namespace ptcr {

#endif // #ifndef SNARK_ACTUATORS_QUICKSET_ptcr_PROTOCOL_H_
