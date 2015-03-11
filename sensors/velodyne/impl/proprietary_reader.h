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


#ifndef SNARK_SENSORS_VELODYNE_PROPRIETARYREADER_H_
#define SNARK_SENSORS_VELODYNE_PROPRIETARYREADER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <iostream>
#include <fstream>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

namespace snark {

/// a simple pcap wrapper
/// @todo move to a more appropriate place, once we figure out where
class proprietary_reader : public boost::noncopyable
{
    public:
        /// constructor, open a file, default stdin
        proprietary_reader( const std::string& filename = "-" );
    
        /// destructor, close file
        ~proprietary_reader();
    
        /// read and return pointer to the current packet; NULL, if end of file
        const char* read();
    
        /// close
        void close();
    
        /// return current timestamp
        boost::posix_time::ptime timestamp() const;
    
    private:
        enum
        {
              headerSize = 16
            , timestampSize = 12
            , payload_size = 1206
            , footerSize = 4
            , packetSize = headerSize + timestampSize + payload_size + footerSize
            , packetNum = 1 // , packetNum = 10 : does not seem to make big impact on the i/o performance
        };
        boost::array< char, packetSize * packetNum > m_buffer;
        std::size_t m_offset;
        std::size_t m_end;
        boost::posix_time::ptime m_timestamp;
        boost::scoped_ptr< std::ifstream > m_ifstream;
        std::istream* m_istream;
};

} 

#endif /*SNARK_SENSORS_VELODYNE_PROPRIETARYREADER_H_*/
