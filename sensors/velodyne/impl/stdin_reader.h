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

#ifndef SNARK_SENSORS_VELODYNE_STDIN_READER_H_
#define SNARK_SENSORS_VELODYNE_STDIN_READER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/array.hpp>
#include <comma/base/types.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace snark {

/// stdin reader
class stdin_reader
{
    public:
        /// constructor
        stdin_reader();

        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();
        
        /// return current timestamp
        const boost::posix_time::ptime& timestamp() const;

    private:
        enum{ payload_size = 1206 };
        comma::uint64 m_microseconds;
        boost::array< char, payload_size > m_packet;
        boost::posix_time::ptime m_timestamp;
        boost::posix_time::ptime m_epoch;
};

} // namespace snark {

#endif /*SNARK_SENSORS_VELODYNE_STDIN_READER_H_*/

