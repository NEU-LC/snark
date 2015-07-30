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


#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <comma/base/exception.h>
#include <fstream>
#include "jai.h"

namespace snark { namespace camera {

struct jai::impl
{
    impl( const std::string& address, const jai::attributes_type& attributes ) : address( address )
    {
        // todo
    }
    
    std::pair< boost::posix_time::ptime, cv::Mat > read()
    {
        // todo
        return std::pair< boost::posix_time::ptime, cv::Mat >();
    }
    
    void close()
    { 
        // todo
    }

    bool closed() const
    { 
        // todo
        return true;
    }
    
    unsigned long total_bytes_per_frame() const
    {
        // todo
        return 0;
    }
    
    void set( const jai::attributes_type& attributes )
    { 
        // todo
    }
    
    jai::attributes_type attributes() const
    { 
        // todo
        return jai::attributes_type();
    }
    
    std::string address;
};

jai::jai( const std::string& address, const jai::attributes_type& attributes ) : pimpl_( new impl( address, attributes ) ) {}

jai::~jai() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > jai::read() { return pimpl_->read(); }

void jai::close() { pimpl_->close(); }

bool jai::closed() const { return pimpl_->closed(); }

//std::vector< XDeviceInformation > jai::list_cameras() { return jai::impl::list_cameras(); }

const std::string& jai::address() const { return pimpl_->address; }

unsigned long jai::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

jai::attributes_type jai::attributes() const { return pimpl_->attributes(); }

void jai::set(const jai::attributes_type& attributes ) { pimpl_->set( attributes ); }

} } // namespace snark{ namespace camera{
