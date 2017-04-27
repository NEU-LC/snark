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


#ifndef SNARK_SENSORS_FLYCAPTURE_H_
#define SNARK_SENSORS_FLYCAPTURE_H_

#include <memory>
#include "FlyCapture2.h"
// #include "attributes.h"
// #include "helpers.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <boost/bimap.hpp>
#include <opencv2/core/core.hpp>


namespace snark { namespace cameras { namespace flycapture {

struct moment {
    // possible values for the '--timestamp=' option
    enum moments {
        none = 0,
        before,
        after,
        average,
    };

    moments value;
    moment( const std::string & s ) : value( s == "before" ? before : ( s == "after" ? after : ( s == "average" ? average : none ) ) ) {
        if ( value == none ) { COMMA_THROW( comma::exception, "moment is not one of '" << list() << "'" ); }
    }

    operator std::string() const
    {
        switch( value )
        {
            case(none):    return "none";
            case(before) : return "before";
            case(after)  : return "after";
            case(average): return "average";
        }
    }
    static std::string list() { return "before,after,average"; }
};

/// image acquisition from flycapture camera
class camera
{
    public:
        /// attributes map type
        // typedef std::map< std::string, std::string > attributes_type;
        typedef std::vector< std::pair<std::string, std::string> > attributes_type;
        typedef std::pair< boost::posix_time::ptime, cv::Mat > frame_pair;
        typedef std::unique_ptr<FlyCapture2::CameraBase> camerabase_ptr;
        
        /// constructor; default id: connect to any available camera
        camera( unsigned int id = 0, const attributes_type& attributes = attributes_type());

        /// destructor
        ~camera();

        /// return attributes
        attributes_type attributes() const;

        /// get timestamped frame
        frame_pair read( const snark::cameras::flycapture::moment & when );
        
        // void test();

        /// return camera id
        unsigned int id() const;

        /// return total bytes per frame
        unsigned long total_bytes_per_frame() const;

        /// close
        void close();

        /// list cameras
        static std::vector< unsigned int > list_camera_serials();

        static FlyCapture2::InterfaceType get_camera_interface(unsigned int serial);

        static const std::string describe_camera(unsigned int serial);

/// multicam
        class multicam
        {
            public:
                typedef std::pair<uint, const camera::attributes_type&> camera_pair;
                typedef std::pair<boost::posix_time::ptime, std::vector<cv::Mat>> frames_pair;

                /// constructor: start capture, call multicam on frame update
                multicam( std::vector<camera_pair>& cameras, const std::vector< unsigned int >& offsets );

                /// destructor: stop capture
                ~multicam();

                /// synchronous trigger of all cameras
                void trigger();

                /// return the images and timestamp
                frames_pair read( const snark::cameras::flycapture::moment & when, bool use_software_trigger = true );
                
                /// return true, if multicam status is ok
                bool good() const;

                /// return the number of cameras in this instance
                unsigned int num_cameras() const;
                
            private:
                /// implementation class, a hack: has to be public to use pvAPI multicam, sigh...
                class impl;
                friend class camera;
                impl* pimpl_;
        };
        
    private:
        friend class multicam::impl;
        class impl;
        std::unique_ptr< impl > pimpl_;
};

} } } // namespace snark { namespace cameras { namespace flycapture {

#endif // SNARK_SENSORS_FLYCAPTURE_H_
