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


#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <comma/base/exception.h>
#include "./gobi.h"

//static void PVDECL pv_callback_( tPvFrame *frame );

namespace snark{ namespace camera{

//static const unsigned int timeOutFactor = 3;
//static const unsigned int maxRetries = 15; 
    
static const unsigned int xenicsPropertyMaxLen = 128;    
    
static const char *xenics_error_to_string_( ErrCode error )
{
    switch( error )
    {
       // error code messages are taken from XCamera.h
        case I_OK: return "Success";
        case I_DIRTY: return "Internal";
        case E_BUG: return "Generic";
        case E_NOINIT: return "Camera was not successfully initialised";
        case E_LOGICLOADFAILED: return "Invalid logic file";
        case E_INTERFACE_ERROR: return "Command interface failure";
        case E_OUT_OF_RANGE: return "Provided value is incapable of being produced by the hardware";
        case E_NOT_SUPPORTED: return "Functionality not supported by this camera";
        case E_NOT_FOUND: return "File/Data not found.";
        case E_FILTER_DONE: return "Filter has finished processing, and will be removed";
        case E_NO_FRAME: return "A frame was requested by calling GetFrame, but none was available";
        case E_SAVE_ERROR: return "Couldn't save to file";
        case E_MISMATCHED: return "Buffer size mismatch";
        case E_BUSY: return "The API can not read a temperature because the camera is busy";
        case E_INVALID_HANDLE: return "An unknown handle was passed to the C API";
        case E_TIMEOUT: return "Operation timed out";
        case E_FRAMEGRABBER: return "Frame grabber error";
        case E_NO_CONVERSION: return "GetFrame could not convert the image data to the requested format";
        case E_FILTER_SKIP_FRAME: return "Filter indicates the frame should be skipped.";
        case E_WRONG_VERSION: return "Version mismatch";
        case E_PACKET_ERROR: return "The requested frame cannot be provided because at least one packet has been lost";
        case E_WRONG_FORMAT: return "The emissivity map you tried to set should be a 16 bit grayscale png";
        case E_WRONG_SIZE: return "The emissivity map you tried to set has the wrong dimensions (w,h)";
        case E_CAPSTOP: return "Internal";
        case E_RFU: return "E_RFU"; // no message is specified in XCamera.h for this case
        default : return "unknown error";
    };
}


static void xenics_set_attribute_( XCHANDLE& handle, const std::string& property_name, const std::string& value )
{
    XPropType property_type;
    if( XC_GetPropertyType(handle, property_name.c_str(), &property_type) != I_OK ) 
    { 
        COMMA_THROW( comma::exception, "failed to get property type for \"" << property_name << "\" in xenics_set_attribute_ (or " << property_name << " is unavailable)" ); 
    }
    ErrCode error_code;
    switch( property_type & XType_Base_Mask )
    {
        case XType_Base_Number:
            // xenics API cannot tell if the property value is long or double, so
            // value is interpreted as double if it has a decimal point, otherwise it is interpreted as long
            if( !( value.find('.') == std::string::npos ) ) error_code = XC_SetPropertyValueF(handle, property_name.c_str(), boost::lexical_cast< double >( value ), "");
            else error_code = XC_SetPropertyValueL(handle, property_name.c_str(), boost::lexical_cast< long >( value ), "");
            break;
        case XType_Base_Enum:
        case XType_Base_Bool:
        {
            error_code = XC_SetPropertyValueL(handle, property_name.c_str(), boost::lexical_cast< long >( value ), "");
            break;
        }
        case XType_Base_String:
            error_code = XC_SetPropertyValue(handle, property_name.c_str(), value.c_str(), "");
            break;
        case XType_Base_Blob:
        default:
            COMMA_THROW( comma::exception, "unknown (or blob) property type for attribute \"" << property_name << "\"" );
    };
    if( error_code != I_OK ) { COMMA_THROW( comma::exception, "failed to set attribute \"" << property_name << "\": " << xenics_error_to_string_( error_code ) << " (" << error_code << ")" ); }
}

static std::string xenics_get_attribute_( XCHANDLE& handle, const char* property_name )
{
    XPropType property_type;
    if( XC_GetPropertyType(handle, property_name, &property_type) != I_OK ) 
    { 
        COMMA_THROW( comma::exception, "failed to get property type for \"" << property_name << "\" in xenics_get_attribute_ (or " << property_name << " is unavailable)" ); 
    }
    switch( property_type & XType_Base_Mask )
    {
        case XType_Base_Number:
        {
            // using floating point version for all properties since xenics API does not tell if the property value is long or double (XC_GetPropertyValueL can be used if value is an integer)
            double f = 0;
            XC_GetPropertyValueF(handle, property_name, &f);
            return boost::lexical_cast< std::string >( f );          
        }
        case XType_Base_Enum:
        case XType_Base_Bool:
        {
            long n = 0;
            XC_GetPropertyValueL(handle, property_name, &n);
            return boost::lexical_cast< std::string >( n );
        }
        case XType_Base_String:
        {
            char buf[xenicsPropertyMaxLen];
            XC_GetPropertyValue(handle, property_name, buf, xenicsPropertyMaxLen);
            return std::string( buf );
        }        
        case XType_Base_Blob:
        default:
            return "";           
    };
}

gobi::attributes_type xenics_attributes_( XCHANDLE& handle )
{  
    gobi::attributes_type attributes;
    int property_count = XC_GetPropertyCount( handle );
    for( int i = 0; i < property_count; ++i ) 
    {
        char property_name[xenicsPropertyMaxLen];
        ErrCode error_code = XC_GetPropertyName(handle, i, &property_name[0], xenicsPropertyMaxLen);
        if( error_code != I_OK ) { COMMA_THROW( comma::exception, "failed to get property name for i=" << i <<" in xenics_attributes_: " << xenics_error_to_string_( error_code ) << " (" << error_code << ")" ); }     
        attributes.insert( std::make_pair( property_name, xenics_get_attribute_( handle, property_name ) ) ); 
    }
    return attributes;    
}

/*
static cv::Mat pv_as_cvmat_( const tPvFrame& frame )
{
    if( frame.Status != ePvErrSuccess ) { return cv::Mat(); }
    int type;
    switch( frame.Format )
    {
        case ePvFmtMono8:
        case ePvFmtBayer8:
            type = CV_8UC1;
            break;
        case ePvFmtBayer16:
        case ePvFmtMono16:
            type = CV_16UC1;
            break;
        case ePvFmtRgb24:
        case ePvFmtBgr24:
            type = CV_8UC3;
            break;
        case ePvFmtRgba32:
        case ePvFmtBgra32:
            type = CV_8UC4;
            break;
        case ePvFmtRgb48:
            type = CV_16UC3;
            break;
        case ePvFmtYuv411:
        case ePvFmtYuv422:
        case ePvFmtYuv444:
            COMMA_THROW( comma::exception, "unsupported format " << frame.Format );
        default:
            COMMA_THROW( comma::exception, "unknown format " << frame.Format );
    };
    return cv::Mat( frame.Height, frame.Width, type, frame.ImageBuffer );
}
*/

class gobi::impl
{
    public:
        impl( std::string address, const attributes_type& attributes ) :
            address_( address ),
            started_( false ),
            timeOut_( 1000 )
        {
            if( address_.empty() ) { COMMA_THROW( comma::exception, "expected camera address, got empty string" ); }
/*            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?           
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            unsigned int size = 0;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                const std::vector< XDeviceInformation >& list = list_cameras();
                size = list.size();
                for( unsigned int i = 0; i < list.size(); ++i ) 
                {
                    if( std::string(list[i].transport) == "GigEVision"
                        && list[i].state == XDS_Available
                        && ( address.empty() || address == list[i].address ) )
                    {
                        address_ = list[i].address;
                        break;
                    }
                }
                if( address_ ) { break; }
            }
            if( !address_ ) { COMMA_THROW( comma::exception, "timeout; camera not found" ); }
            if( address.empty() && size > 1 )
            {
                const std::vector< XDeviceInformation >& list = list_cameras();
                std::stringstream stream;
                for( std::size_t i = 0; i < list.size(); ++i ) // todo: serialize properly with name-value
                {
                    stream << "id=" << list[i].pid << "," << "name=\"" << list[i].name << "\"" << "," << "serial=\"" << list[i].serial << "\"" << "," << "address=\"" << list[i].address << "\"" << std::endl;
                }
                COMMA_THROW( comma::exception, "no address provided and multiple cameras found: would pick up a random one\n\tavailable cameras:\n" << stream.str() );
            } 
*/
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;           
            std::string camera_name = "gev://" + address_;
            handle_ = XC_OpenCamera( camera_name.c_str() );
            for( ; ( !XC_IsInitialised(handle_) ) && ( now < end ); now = boost::posix_time::microsec_clock::universal_time() )
            {
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
                handle_ = XC_OpenCamera( camera_name.c_str() );
            }
            if( !XC_IsInitialised(handle_) ) { close(); COMMA_THROW( comma::exception, "failed to open gobi camera " + camera_name ); }

            set( attributes );
/*
            PvAttrEnumSet( handle_, "AcquisitionMode", "Continuous" );
            PvAttrEnumSet( handle_, "FrameStartTriggerMode", "FixedRate" );
            ::memset( &frame_, 0, sizeof( tPvFrame ) ); // voodoo
            result = PvAttrUint32Get( handle_, "TotalBytesPerFrame", &total_bytes_per_frame_ );
            if( result != ePvErrSuccess ) { close(); COMMA_THROW( comma::exception, "failed to get TotalBytesPerFrame from gobi camera " << *id_ << ": " << xenics_error_to_string_( result ) << " (" << result << ")" ); }
            buffer_.resize( total_bytes_per_frame_ );
            frame_.ImageBuffer = &buffer_[0];
            frame_.ImageBufferSize = total_bytes_per_frame_;
            float frameRate;
            PvAttrFloat32Get( handle_, "FrameRate", &frameRate);
            timeOut_ = 1000 / frameRate;
            timeOut_ *= timeOutFactor;
*/
        }
        
        void set( const attributes_type& attributes )
        {
             for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
            {
                xenics_set_attribute_( handle_, i->first, i->second );
            }     
        }
        
        ~impl() { close(); }

        void close()
        {
            address_ = "";
            if( XC_IsInitialised(handle_)  ) { XC_CloseCamera( handle_ ); }
        }
        
        std::pair< boost::posix_time::ptime, cv::Mat > read()
        {
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            bool success = false;
/*          
            unsigned int retries = 0;
            while( !success && retries < maxRetries )
            {
                tPvErr result = ePvErrSuccess;
                if( !started_ )
                {
                    result = PvCaptureStart( handle_ );
                }
                if( result == ePvErrSuccess )
                {
                    result = PvCaptureQueueFrame( handle_, &frame_, 0 );
                    if( result == ePvErrSuccess )
                    {
                        if( !started_ )
                        {
                            result = PvCommandRun( handle_, "AcquisitionStart" );
                            started_ = true;
                        }
                        if( result == ePvErrSuccess )
                        {
                            result = PvCaptureWaitForFrameDone( handle_, &frame_, timeOut_ );
                            if( result == ePvErrSuccess )
                            {
                                result = frame_.Status;
                            }
                            pair.first = boost::posix_time::microsec_clock::universal_time();
                        }
                    }
                }
                if( result == ePvErrSuccess )
                {
                    pair.second = pv_as_cvmat_( frame_ );
                    success = true;
                }
                else if( result == ePvErrDataMissing || result == ePvErrTimeout )
                {
                    PvCaptureQueueClear( handle_ );
                    PvCommandRun( handle_, "AcquisitionStop" );
                    PvCaptureEnd( handle_ );
                    started_ = false;
                }
                else
                {
                    COMMA_THROW( comma::exception, "got frame with invalid status on camera " << *id_ << ": " << xenics_error_to_string_( result ) << "(" << result << ")" );
                    close();                    
                }
                retries++;
            }
*/            
            if( success ) { return pair; }
            COMMA_THROW( comma::exception, "got lots of missing frames or timeouts" << std::endl << std::endl << "it is likely that MTU size on your machine is less than packet size" << std::endl << "check PacketSize attribute (gobi-cat --list-attributes)" << std::endl << "set packet size (e.g. gobi-cat --set=PacketSize=1500)" << std::endl << "or increase MTU size on your machine" );
        }

        const XCHANDLE& handle() const { return handle_; }
        
        XCHANDLE& handle() { return handle_; }

        std::string address() const { return address_; }

//        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }

        static std::vector< XDeviceInformation > list_cameras()
        {
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            std::vector< XDeviceInformation > devices;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                unsigned int device_count = 0;
                XCD_EnumerateDevices(NULL, &device_count, XEF_GigEVision);
                if (device_count == 0) { break; }
                devices.resize( device_count );
                XCD_EnumerateDevices(&devices[0], &device_count, XEF_UseCached);
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
            }
            return devices;
        }
        
        static std::string format_camera_info(const XDeviceInformation& device)
        {
            std::string state = device.state == XDS_Available ? "Available" : device.state == XDS_Busy ? "Busy" : "Unreachable";
            std::stringstream output;
            output << "id=" << device.pid << "," 
                   << "name=\"" << device.name << "\"" << "," 
                   << "serial=\"" << device.serial << "\"" << "," 
                   << "address=\"" << device.address << "\"" << ","
                   << "transport=\"" << device.transport << "\"" << "," 
                   << "state=\"" << state << "\"";
            return output.str();
        }
    private:
//        friend class gobi::callback::impl;
        XCHANDLE handle_;
//        tPvFrame frame_;
//        std::vector< char > buffer_;
        std::string address_;
//        boost::optional< std::string > address_;
//        boost::optional< unsigned int > id_;
//        unsigned long total_bytes_per_frame_;
        bool started_;
        unsigned int timeOut_; // milliseconds        
};

/*
class gobi::callback::impl
{
    public:
        typedef boost::function< void ( const std::pair< boost::posix_time::ptime, cv::Mat >& ) > OnFrame;
        
        impl( gobi& gobi, OnFrame on_frame )
            : on_frame( on_frame )
            , handle( gobi.pimpl_->handle() )
            , frame( gobi.pimpl_->frame_ )
            , good( true )
            , is_shutdown( false )
        {
            tPvErr result;
            PvCaptureQueueClear( handle );
            frame.Context[0] = this;
            result = PvCaptureStart( handle );
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to start capturing on camera " << gobi.pimpl_->id() << ": " << xenics_error_to_string_( result ) << " (" << result << ")" ); }
            result = PvCaptureQueueFrame( handle, &frame, pv_callback_ );
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to set capture queue frame on camera " << gobi.pimpl_->id() << ": " << xenics_error_to_string_( result ) << " (" << result << ")" ); }
            result = PvCommandRun( handle, "AcquisitionStart" );
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to start acquisition on camera " << gobi.pimpl_->id() << ": " << xenics_error_to_string_( result ) << " (" << result << ")" ); }
        }

        ~impl()
        {
            is_shutdown = true;
            PvCommandRun( handle, "Acquisitionstop" );
            PvCaptureQueueClear( handle );
            PvCaptureEnd( handle );
        }

        OnFrame on_frame;
        tPvHandle& handle;
        tPvFrame& frame;
        bool good;
        bool is_shutdown;
};
*/

} } // namespace snark{ namespace camera{

/*
static void PVDECL pv_callback_( tPvFrame *frame )
{
    snark::camera::gobi::callback::impl* c = reinterpret_cast< snark::camera::gobi::callback::impl* >( frame->Context[0] );
    if( c->is_shutdown ) { return; }
    std::pair< boost::posix_time::ptime, cv::Mat > m( boost::posix_time::microsec_clock::universal_time(), cv::Mat() );
    if( frame ) { m.second = snark::camera::pv_as_cvmat_( *frame ); }
    c->on_frame( m );
    tPvErr result = PvCaptureQueueFrame( c->handle, &c->frame, pv_callback_ );
    if( result != ePvErrSuccess ) { c->good = false; }
}
*/

namespace snark{ namespace camera{

gobi::gobi( std::string address, const gobi::attributes_type& attributes ) : pimpl_( new impl( address, attributes ) ) {}

gobi::~gobi() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > gobi::read() { return pimpl_->read(); }

void gobi::close() { pimpl_->close(); }

std::vector< XDeviceInformation > gobi::list_cameras() { return gobi::impl::list_cameras(); }

std::string gobi::format_camera_info( const XDeviceInformation& device ) { return gobi::impl::format_camera_info( device ); }

std::string gobi::address() const { return pimpl_->address(); }

//unsigned long gobi::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

gobi::attributes_type gobi::attributes() const { return xenics_attributes_( pimpl_->handle() ); }

/*
gobi::callback::callback( gobi& gobi, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame )
    : pimpl_( new callback::impl( gobi, on_frame ) )
{
}
*/

//gobi::callback::~callback() { delete pimpl_; }

//bool gobi::callback::good() const { return pimpl_->good; }

} } // namespace snark{ namespace camera{
