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

#ifndef SNARK_SENSORS_VELODYNE_UDPREADER_H_
#define SNARK_SENSORS_VELODYNE_UDPREADER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/array.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>

namespace snark { 

/// udp reader
class udp_reader : public boost::noncopyable
{
    public:
        /// constructor
        udp_reader( unsigned short port );

        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();

        /// close
        void close();

        /// return current timestamp
        const boost::posix_time::ptime& timestamp() const;

    private:
        boost::asio::io_service service_;
        boost::asio::ip::udp::socket socket_;
        boost::array< char, 2000 > packet_; // way greater than velodyne packet
        boost::posix_time::ptime timestamp_;
};

} // namespace snark {

#endif /*SNARK_SENSORS_VELODYNE_UDPREADER_H_*/

