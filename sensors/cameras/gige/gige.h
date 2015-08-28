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


#ifndef SNARK_SENSORS_GIGE_H_
#define SNARK_SENSORS_GIGE_H_

#include <PvApi.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

#include <opencv2/core/core.hpp>


namespace snark{ namespace camera{

/// image acquisition from gige camera
class gige
{
    public:
        /// attributes map type
        typedef std::map< std::string, std::string > attributes_type;

        /// constructor; default id: connect to any available camera
        gige( unsigned int id = 0, const attributes_type& attributes = attributes_type() );
        
        /// constructor; default id: connect to any available camera
        gige( unsigned int id, bool use_camera_timestamp, const attributes_type& attributes = attributes_type() );

        /// destructor
        ~gige();

        /// return attributes
        attributes_type attributes() const;

        /// get timestamped frame
        std::pair< boost::posix_time::ptime, cv::Mat > read();

        /// return camera id
        unsigned int id() const;

        /// set frame timeout manually
        void set_frame_timeout( unsigned int timeout );

        /// get frame timeout
        unsigned int frame_timeout() const;

        /// return total bytes per frame
        unsigned long total_bytes_per_frame() const;

        /// close
        void close();

        /// list cameras
        static std::vector< tPvCameraInfo > list_cameras();

        /// callback
        class callback
        {
            public:
                /// constructor: start capture, call callback on frame update
                callback( gige& gige, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame );

                /// destructor: stop capture
                ~callback();

                /// implementation class, a hack: has to be public to use pvAPI callback, sigh...
                class impl;

                /// return true, if callback status is ok
                bool good() const;

            private:
                friend class gige;
                impl* pimpl_;
        };

    private:
        friend class callback::impl;
        class impl;
        impl* pimpl_;
};

} } // namespace snark{ namespace camera{

#endif // SNARK_SENSORS_GIGE_H_
