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

#pragma once

#include <boost/optional.hpp>
#include "laser_return.h"
#include "scan_tick.h"

namespace snark { namespace velodyne {

struct stream
{
    virtual ~stream() {}
    virtual laser_return* read() = 0;
    virtual void skip_scan() = 0;
    virtual unsigned int scan() const = 0;
    virtual void close() = 0;
    virtual bool is_scan_valid() { return true; }
    
    /// set missing packets threshold in scan_tick
    /// @param threshold_n number of consecutive packets that is used in scan_tick to break into new scan and mark it as invalid
    void set_missing_packets_threshold(const boost::optional<unsigned>& threshold_n) { scan_tick_.threshold_n = threshold_n; }
    
    virtual unsigned packet_duration() = 0;
    double packet_angle(double rotation_per_second) { return 360 / ( 1000000.0 /(rotation_per_second *packet_duration())); }
protected:
    scan_tick scan_tick_;
};

} } // namespace snark {  namespace velodyne {
