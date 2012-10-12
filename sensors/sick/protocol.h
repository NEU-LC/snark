// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_SENSORS_SICK_PROTOCOL_H_
#define SNARK_SENSORS_SICK_PROTOCOL_H_

/// @file protocol.h
/// sick (ibeo) ldmrs laser communication protocol
/// @author vsevolod vlaskine (v.vlaskine@acfr.usyd.edu.au)

#include <iostream>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <snark/sensors/sick/packets.h>

namespace snark {  namespace sick { namespace ldmrs {

class protocol : public boost::noncopyable
{
    public:
        /// constructor for two-way communication with sensor
        protocol( std::iostream& stream );

        /// constructor for reading only (scans, warnings, etc)
        protocol( std::istream& stream );
        
        /// destructor
        ~protocol();
        
        /// reset sensor (send reset DSP)
        void reset_dsp();
        
        /// send command
        template < typename command > typename command::response write( const command& c );
        
        /// read scan data packet
        /// @note once scanning started, call readscan() often to flush receive buffer
        const scan_packet* readscan();
        
        /// return last fault, if any
        boost::optional< fault > last_fault();
        
        /// fault exception, quick and dirty (since comma::exception is so difficult to inherit from)
        struct faultException : public std::exception {};
        
    private:
        class impl;
        impl* m_pimpl;
};

} } } // namespace snark {  namespace sick { namespace ldmrs {

#endif // #ifndef SNARK_SENSORS_SICK_PROTOCOL_H_
