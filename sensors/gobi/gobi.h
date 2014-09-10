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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#ifndef SNARK_SENSORS_GOBI_H_
#define SNARK_SENSORS_GOBI_H_

#include <XCamera.h>
#include <XFooters.h>
#include <XFilters.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

#include <opencv2/core/core.hpp>

namespace snark{ namespace camera{

class gobi
{
    public:
        typedef std::map< std::string, std::string > attributes_type;       
        
        gobi( std::string address, const attributes_type& attributes = attributes_type() );

        ~gobi();

        attributes_type attributes() const;
        
        void set( const attributes_type& attributes );

        std::pair< boost::posix_time::ptime, cv::Mat > read();

        std::string address() const;
        
        std::string temperature_unit() const;

        unsigned long total_bytes_per_frame() const;
        
        void close();
        
        bool closed() const;

        static std::vector< XDeviceInformation > list_cameras();
        
        static std::string format_camera_info(const XDeviceInformation& camera_info);
        
        void enable_thermography( std::string temperature_unit, std::string calibration_file );
        
        void disable_thermography();
        
    private:
        class impl;
        impl* pimpl_;
};

} } // namespace snark{ namespace camera{

#endif // SNARK_SENSORS_GOBI_H_
