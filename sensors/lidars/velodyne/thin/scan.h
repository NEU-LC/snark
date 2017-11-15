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


#ifndef SNARK_SENSORS_VELODYNE_THIN_SCAN
#define SNARK_SENSORS_VELODYNE_THIN_SCAN

#include "../hdl64/packet.h"
#include "../scan_tick.h"

namespace snark {  namespace velodyne { namespace thin {

class scan
{
public:
    scan();
    void thin( velodyne::hdl64::packet& packet, double rate, double angularSpeed );
    bool empty() const { return m_empty; }
private:
    enum { m_size = 12 * 32 };
    struct index
    {
        unsigned int idx;
        unsigned int block;
        unsigned int laser;
        index() : idx( 0 ), block( 0 ), laser( 0 ) {}
        const index& operator++()
        {
            ++idx;
            if( block & 0x1 )
            {
                ++laser;
                if( laser < 32 ) { --block; } else { laser = 0; ++block; }
            }
            else
            {
                ++block;
            }
            return *this;
        }
        bool operator==( const index& rhs ) const { return idx == rhs.idx; }
    };
    unsigned int m_scan;
    scan_tick m_tick;
    bool m_closed;
    unsigned int m_output;
    bool m_outputCurrentscan;
    bool m_empty; /// true if the last packet is beeing entirely dropped
};


} } } // namespace snark {  namespace velodyne { namespace thin {

#endif // SNARK_SENSORS_VELODYNE_THIN_SCAN

