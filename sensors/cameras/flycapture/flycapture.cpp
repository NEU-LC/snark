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

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/assign.hpp>
#include <boost/bimap.hpp>
#include <boost/bind.hpp>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include "flycapture.h"
#include "helpers.h"
#include "attributes.h"

/*TODO:
* Discard argument is ignored.
* implement a callback solution
*/

boost::posix_time::ptime
midpoint(boost::posix_time::ptime const& a, boost::posix_time::ptime const& b) {
    return a + (b-a)/2;
}

namespace snark{ namespace camera{ 

    static const unsigned int max_retries = 15;

    bool flycapture_collect_frame_(cv::Mat & image, FlyCapture2::CameraBase* handle, bool & started)
    {
        FlyCapture2::Error result;
        FlyCapture2::Image frame_;
        FlyCapture2::Image raw_image;

        if( !handle ) { COMMA_THROW(comma::exception, "read from bad camera"); }
        result = handle->RetrieveBuffer(&raw_image);
        frame_.DeepCopy(&raw_image);
        raw_image.ReleaseBuffer();

        if( result == FlyCapture2::PGRERROR_OK ) 
        {
            cv::Mat cv_image( frame_.GetRows(), frame_.GetCols(), flycapture_get_cv_type_( frame_ ), frame_.GetData() );
            cv_image.copyTo(image);
            return true;
        }
        else if( result == FlyCapture2::PGRERROR_IMAGE_CONSISTENCY_ERROR )
        {
//report damaged frame and try again
            std::cerr << "flycapture-cat error: " << result.GetDescription() << ". Retrying..." << std::endl;
            return false; 
        }
        else if( ( result == FlyCapture2::PGRERROR_ISOCH_START_FAILED ) 
            || ( result == FlyCapture2::PGRERROR_ISOCH_ALREADY_STARTED )
            || ( result == FlyCapture2::PGRERROR_ISOCH_NOT_STARTED ) )
        {
//These are errors that result in a retry
            std::cerr << "flycapture-cat error: " << result.GetDescription() << ". Restarting camera." << std::endl;
            handle->StopCapture();
            started = false;
            return false;
        }
        else 
        {  
            COMMA_THROW( comma::exception, "got frame with invalid status " << result.GetType() << ": " << result.GetDescription() );
        }
    }

    class flycapture::impl
    {
//Note, for Point Grey, the serial number is used as ID
    public:
        impl( unsigned int id, const attributes_type& attributes ) :
        handle_(nullptr), id_(id), started_( false ), timeOut_( 1000 )
        {
            initialize_();
            FlyCapture2::PixelFormat pixel_format;
            uint width, height;
            std::vector< unsigned int > camera_list = list_camera_serials();
// Check if id is in the camera enumeration
            if(std::find(camera_list.begin(), camera_list.end(), id) == camera_list.end())
                { COMMA_THROW(comma::exception, "couldn't find camera with serial " + std::to_string(id)); }

            if (!connect(id, handle_, guid_)) 
            { COMMA_THROW(comma::exception, "couldn't connect to serial " + std::to_string(id)); }
            // start();
//Get Point grey unique id (guid) from serial number. guid does not exist in CameraInfo, and so it does not appear in the camera list

            if(FlyCapture2::GigECamera* camera_gige = dynamic_cast<FlyCapture2::GigECamera*>(handle_))
            {
                for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
                {
                    boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );
                    flycapture_set_attribute_( camera_gige, i->first, i->second );
                }   
            }
            else if(FlyCapture2::Camera* camera_serial = dynamic_cast<FlyCapture2::Camera*>(handle_))
            {
                for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
                {
                    boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );
                    flycapture_set_attribute_( camera_serial, i->first, i->second );
                }
            }
            else {
                COMMA_THROW( comma::exception, "Unsupported camera type" );
            }

            width = boost::lexical_cast<uint>( flycapture_get_attribute_( handle_, "width" ) );
            height = boost::lexical_cast<uint>( flycapture_get_attribute_( handle_, "height" ) );
            pixel_format = get_pixel_format_map()->right.at( flycapture_get_attribute_( handle_,"PixelFormat" ) );
            total_bytes_per_frame_ = width * height * flycapture_bits_per_pixel_( pixel_format ) / 8;
        }

        ~impl() { close(); }

        void start()
        {
            // FlyCapture2::Error error;
            // const boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( 150 );
            // boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            // started_ = false;
            // for (uint i = max_retries; !started_ && i > 0; --i)
            // {
            //     std::cerr << "Trying to connect..." << std::endl;
            //     boost::thread::sleep( now + timeout );
            // }
            // if (!started_) { COMMA_THROW( comma::exception, "failed to open camera: " << id()); }
            // handle_->StartCapture();
        }

        void close()
        {
            if(handle_) {
                id_.reset();
                if( handle_->IsConnected() ) {
                    handle_->StopCapture();
                    handle_->Disconnect();
                }
                delete handle_;
            }
        }

        std::pair< boost::posix_time::ptime, cv::Mat > read()
        {
            cv::Mat image_;
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            bool success = false;
            unsigned int retries = 0;

            while( !success && retries < max_retries )
            {
                FlyCapture2::Error result;
                if( !started_ ) { result = handle_->StartCapture(); }
// error is not checked as sometimes the camera will start correctly but return an error
                started_ = true;
                std::cerr << "read::inner()" << std::endl;
                success = flycapture_collect_frame_( image_, handle_ , started_ );
                pair.first = boost::posix_time::microsec_clock::universal_time();
                if( success ) { pair.second = image_; }
                retries++;
            }
            if( success ) { return pair; }
            COMMA_THROW( comma::exception, "Got lots of missing frames or timeouts... check MTU and packet size" << std::endl)
        }

        const FlyCapture2::CameraBase* handle() const { return handle_; }

        FlyCapture2::CameraBase* handle() { return handle_; }

        unsigned int id() const { return *id_; }

        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }

        static std::vector< unsigned int > list_camera_serials()
        {
            boost::this_thread::sleep( boost::posix_time::milliseconds( 100 ) );
            std::vector< unsigned int > list;
            unsigned int num_cameras = 0;
            flycapture_assert_ok_(
                bus_manager.GetNumOfCameras( &num_cameras ),
                "cannot find point grey cameras"
            );
// USB Cameras
            for (unsigned int i = 0; i < num_cameras; ++i)
            {
                unsigned int serial_number;
                flycapture_assert_ok_(
                    bus_manager.GetCameraSerialNumberFromIndex(i, &serial_number ),
                    "Error getting camera serial"
                );
                list.push_back(serial_number);
            }
// GigE Cameras
            // BUG DiscoverGigECameras will sometimes crash if there are devices on a different subnet
            FlyCapture2::CameraInfo cam_info[num_cameras];
            flycapture_assert_ok_(
                FlyCapture2::BusManager::DiscoverGigECameras( cam_info, &num_cameras ),
                "cannot discover point grey cameras via GiGe interface"
            );
            // If array is not empty, convert to list and exit
            for (unsigned int i = 0; i < num_cameras; ++i)
            {
                list.push_back(cam_info[i].serialNumber);
            }
            return list;
        }

        static void disconnect(FlyCapture2::CameraBase*& base)
        {
            if (base)
            {
                base->StopCapture();
                base->Disconnect();
                delete base;
                base = nullptr;
            }
            else
            { COMMA_THROW(comma::exception, "can't disconnect because camera not connected to this handle"); }
        }

        static bool connect(uint serial, FlyCapture2::CameraBase*& base, FlyCapture2::PGRGuid& guid)
        {
            // Instantiate the right camera type based on the interface
            FlyCapture2::InterfaceType interface;
            if (base)
            { COMMA_THROW(comma::exception, "camera already connected to this handle"); }
            flycapture_assert_ok_(
                bus_manager.GetCameraFromSerialNumber(serial, &guid),
                "cannot find camera with serial " + std::to_string(serial)
            );
            flycapture_assert_ok_(
                bus_manager.GetInterfaceTypeFromGuid( &guid, &interface ),
                "cannot determine interface for camera with serial " + std::to_string(serial)
            );

            switch(interface)
            {
                case FlyCapture2::INTERFACE_USB2:
                case FlyCapture2::INTERFACE_USB3:
                case FlyCapture2::INTERFACE_IEEE1394:
                    base = new FlyCapture2::Camera();
                    break;
                case FlyCapture2::INTERFACE_GIGE:
                    base = new FlyCapture2::GigECamera();
                    break;
                default:
                COMMA_THROW(comma::exception, "unknown interface for camera " + std::to_string(serial));
            }
            FlyCapture2::Error error;
            if( (error = base->Connect(&guid)) != FlyCapture2::PGRERROR_OK)
            {
                std::cerr << "Failed to connect (" << error.GetDescription() << ")" << std::endl;
                return false;
            }
            return true;
        }

        static const std::string describe_camera(unsigned int serial)
        {
            FlyCapture2::CameraInfo camera_info;
            FlyCapture2::CameraBase* camera;
            FlyCapture2::PGRGuid guid;
            FlyCapture2::InterfaceType interface;

            if (!connect(serial, camera, guid))
            { return "serial=" + std::to_string(serial) + ",ERROR"; }

            flycapture_assert_ok_(
                bus_manager.GetInterfaceTypeFromGuid( &guid, &interface ),
                "cannot determine interface for camera with serial " + std::to_string(serial)
            );

            flycapture_assert_ok_(
                camera->GetCameraInfo( &camera_info ),
                "couldn't get camera info for serial " + std::to_string(serial)
            );
            disconnect(camera);

            return 
                "serial=" + std::to_string(camera_info.serialNumber) +
                ",interface=" + get_interface_string(interface) +
                ",model=" + camera_info.modelName +
                ",vendor=" + camera_info.vendorName +
                ",sensor=" + camera_info.sensorInfo +
                ",resolution=" + camera_info.sensorResolution +
                ",version=" + camera_info.firmwareVersion +
                ",build_time=" + camera_info.firmwareBuildTime;
        }

        void software_trigger(bool broadcast)
        {
            // handle_->FireSoftwareTrigger(broadcast);
        }

    private:
        friend class flycapture::callback::impl;
        friend class flycapture::multicam::impl;
        FlyCapture2::CameraBase* handle_;
        boost::optional< unsigned int > id_;
        FlyCapture2::PGRGuid guid_;
        std::vector< char > buffer_;
        uint total_bytes_per_frame_;
        bool started_;
        unsigned int timeOut_; /*milliseconds*/
        static FlyCapture2::BusManager bus_manager;

        static void initialize_() /*quick and dirty*/
        { }
    };

    class flycapture::callback::impl
    {
    public:
        typedef boost::function< void ( const std::pair< boost::posix_time::ptime, cv::Mat >& ) > OnFrame;

        impl( flycapture& flycapture, OnFrame on_frame )
        : on_frame( on_frame ), good( true ), is_shutdown( false )
        {
        }

        ~impl()
        {
            is_shutdown = true;
        }

        OnFrame on_frame;
        bool good;
        bool is_shutdown;
    };

// multicam
    class flycapture::multicam::impl
    {
    public:
        impl(std::vector<camera_pair>& camera_pairs)
        : good( false )
        {
            for (camera_pair& pair : camera_pairs)
            {
                uint serial = pair.first;
                const attributes_type attributes = pair.second;
                std::cerr << "construct impl(" << serial << ", " << attributes.size() << ")" << std::endl;
                boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );
                cameras_.emplace_back( serial, attributes );
                boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );
                std::cerr << "+ impl(" << serial << ", " << attributes.size() << ")" << std::endl;
            }
            if (cameras_.size()) { good = true; }
        }

        ~impl()
        {
            for (int i = cameras_.size() - 1; i >= 0; --i)
            {
                std::cerr << "delete impl(" << cameras_[i].id() << ")" << std::endl;
                boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );
                cameras_.pop_back();
                boost::this_thread::sleep( boost::posix_time::milliseconds( 10 ) );
                std::cerr << "- impl(" << cameras_[i].id() << ")" << std::endl;
            }
        }

        void trigger()
        {
            if (good) { cameras_[0].software_trigger(true); }
        }

        flycapture::multicam::frames_pair read()
        {
            if (!good) { COMMA_THROW(comma::exception, "multicam read without good cameras"); }
            flycapture::multicam::frames_pair pair;

            trigger();
            boost::posix_time::ptime begin = boost::posix_time::microsec_clock::universal_time();
            for (auto& camera : cameras_)
            {
                // pair.second.push_back(camera.read().second);
                const auto& image = camera.read().second;
                std::cerr << "read: " << image.rows << ", " << image.cols << std::endl;
            }
            boost::posix_time::ptime end = boost::posix_time::microsec_clock::universal_time();
            pair.first = midpoint(begin, end);

            return pair;
        }

        bool good;
        std::vector<flycapture::impl> cameras_;
    };
    
} } /*namespace snark{ namespace camera{*/

    namespace snark{ namespace camera{

// flycapture class

        flycapture::flycapture( unsigned int id, const flycapture::attributes_type& attributes) : pimpl_( new impl( id, attributes ) ) {}

        flycapture::~flycapture() { }

        std::pair< boost::posix_time::ptime, cv::Mat > flycapture::read() { return pimpl_->read(); }

        void flycapture::close() { pimpl_->close(); }

        std::vector< unsigned int > flycapture::list_camera_serials() { return flycapture::impl::list_camera_serials(); }

        const std::string flycapture::describe_camera(unsigned int serial) { return flycapture::impl::describe_camera(serial); }

        unsigned int flycapture::id() const { return pimpl_->id(); }

        unsigned long flycapture::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

        flycapture::attributes_type flycapture::attributes() const { return flycapture_attributes_( pimpl_->handle() ); }

// flycapture::callback class
        flycapture::callback::callback(
            flycapture& flycapture,
            boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame )
        : pimpl_( new callback::impl( flycapture, on_frame ) )
        {
        }

        flycapture::callback::~callback() { delete pimpl_; }

        bool flycapture::callback::good() const { return pimpl_->good; }

// flycapture::multicam class
        flycapture::multicam::multicam(std::vector<camera_pair>& cameras)
        : pimpl_( new multicam::impl( cameras ) )
        {
        }

        flycapture::multicam::~multicam() { delete pimpl_; }

        bool flycapture::multicam::good() const { return pimpl_->good; }

        void flycapture::multicam::trigger()
        { pimpl_->trigger(); }

        flycapture::multicam::frames_pair flycapture::multicam::read()
        { return pimpl_->read(); }

} } /* namespace snark{ namespace camera{ */

FlyCapture2::BusManager snark::camera::flycapture::impl::bus_manager;
