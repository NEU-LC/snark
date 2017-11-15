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


#ifndef SNARK_SENSORS_VELODYNE_THIN_READER_H_
#define SNARK_SENSORS_VELODYNE_THIN_READER_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include "../../../../timing/time.h"
#include "../thin/thin.h"

namespace snark {

/// reader for thinned velodyne data
class thin_reader : public boost::noncopyable
{
    public:
        thin_reader();

        const char* read();

        void close();

        boost::posix_time::ptime timestamp() const;

        bool is_new_scan(); // quick and dirty

    private:
        enum { timeSize = 12 };
        char m_buf[ velodyne::thin::maxBufferSize + timeSize ];
        velodyne::hdl64::packet m_packet;
        boost::posix_time::ptime m_timestamp;
        boost::optional< comma::uint32 > last_scan_;
        bool is_new_scan_;
};

}

#endif // SNARK_SENSORS_VELODYNE_THIN_READER_H_
