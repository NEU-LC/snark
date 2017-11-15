// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef SNARK_SENSORS_VELODYNE_STREAM_READER_H_
#define SNARK_SENSORS_VELODYNE_STREAM_READER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <fstream>
#include <iostream>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/base/types.h>
#include "../hdl64/packet.h"

namespace snark {

/// stream reader
class stream_reader
{
    public:
        /// constructor
        stream_reader( std::istream& is = std::cin );

        /// constructor
        stream_reader( const std::string& filename );

        /// destructor
        ~stream_reader();

        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();

        /// return current timestamp
        const boost::posix_time::ptime& timestamp() const;
        
        /// close
        void close();

    private:
        boost::scoped_ptr< std::ifstream > ifstream_;
        std::istream& istream_;
        enum { payload_size = velodyne::hdl64::packet::size };
        comma::uint64 m_microseconds;
        boost::array< char, payload_size > m_packet;
        boost::posix_time::ptime m_timestamp;
        boost::posix_time::ptime m_epoch;
};

} // namespace snark {

#endif /*SNARK_SENSORS_VELODYNE_STREAM_READER_H_*/

