// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#ifndef SNARK_SENSORS_VIMBA_CAMERA_H_
#define SNARK_SENSORS_VIMBA_CAMERA_H_

#include <map>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <VimbaCPP/Include/Camera.h>
#include "frame_observer.h"

namespace snark { namespace vimba {

struct ptp_status
{
    boost::posix_time::ptime t;
    bool use_ptp;
    std::string value;
};


class attribute;

class camera
{
    public:
        typedef std::pair< boost::posix_time::ptime, cv::Mat > timestamped_frame;
        typedef std::map< std::string, std::string> name_values;
        typedef enum
        {
            ACQUISITION_MODE_UNKNOWN,
            ACQUISITION_MODE_CONTINUOUS,
            ACQUISITION_MODE_SINGLE
        } acquisition_mode_t;

        camera( const std::string& camera_id );
        camera( const AVT::VmbAPI::CameraPtr& camera_ptr );
        ~camera();

        name_values info() const;
        std::vector< attribute > attributes() const;
        boost::optional< attribute > get_attribute( const std::string& name ) const;

        void set_feature( const std::string& name, const std::string& value = "" ) const;
        void set_features( const std::string& name_values ) const;

        void set_acquisition_mode( acquisition_mode_t acquisition_mode ) { acquisition_mode_ = acquisition_mode; }
        /// in tests double-buffering seems sufficient but we'll use three frames to
        /// allow for possible jitter in processing time
        void start_acquisition( frame_observer::callback_fn callback, unsigned int num_frames = 3 ) const;
        void stop_acquisition() const;

        timestamped_frame frame_to_timestamped_frame( const snark::vimba::frame& frame, snark::vimba::ptp_status& ptp_status_out ) const;

    private:
        typedef const boost::function< VmbErrorType( std::string& ) > getter_fn;

        static void add_name_value( const char* label, getter_fn fn, name_values& name_value_pairs );

        AVT::VmbAPI::CameraPtr camera_;
        acquisition_mode_t acquisition_mode_;
        mutable VmbUint64_t last_frame_id_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_CAMERA_H_
