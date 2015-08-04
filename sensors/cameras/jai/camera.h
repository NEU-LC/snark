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

/// @author vsevolod vlaskine

#ifndef SNARK_SENSORS_JAI_CAMERA_H_
#define SNARK_SENSORS_JAI_CAMERA_H_

#include <map>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <Jai_Factory.h>

namespace snark { namespace jai {
    
class camera
{
    public:
        struct info
        {
            std::string manufacturer;
            std::string model_name;
            std::string ip_address;
            std::string mac_address;
            std::string serial_number;
            std::string username;
            //std::string interface_id; // seems to be a binary implementation detail
        };
        
        typedef std::map< std::string, std::string > attributes_type;       
        
        ~camera();

        attributes_type attributes() const;
        
        void set( const attributes_type& attributes );
        
        unsigned int width() const;
        
        unsigned int height() const;
        
        unsigned long total_bytes_per_frame() const;
        
        void close();
        
        bool closed() const;
        
        CAM_HANDLE handle();
        
        CAM_HANDLE handle() const;
        
    private:
        friend class factory;
        class impl;
        impl* pimpl_;
        camera( impl* i = NULL );
};

class factory
{
    public:
        factory();
        
        ~factory();
        
        std::vector< std::string > list_devices(); // todo? make const
        
        camera* make_camera( const std::string& id = "" ); // todo? make const
        
        camera::info camera_info( const std::string& id ) const;
        
    private:
        class impl;
        impl* pimpl_;
};

} } // namespace snark{ namespace camera{

#endif // SNARK_SENSORS_JAI_CAMERA_H_
