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
#include "gobi.h"

namespace snark{ namespace camera{

static const unsigned int xenicsPropertyMaxLen = 128;    
static const boost::posix_time::time_duration xenicsTimeout = boost::posix_time::seconds( 5 );
static const boost::posix_time::time_duration xenicsSleepDuration = boost::posix_time::milliseconds( 100 );

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
  
static void xenics_set_attribute_( XCHANDLE& handle, const std::string& name, const std::string& value )
{
    XPropType property_type;
    if( XC_GetPropertyType(handle, name.c_str(), &property_type) != I_OK ) 
    { 
        COMMA_THROW( comma::exception, "failed to get property type for \"" << name << "\" (or " << name << " is unavailable)" ); 
    }
    ErrCode error_code;
    bool mismatched = false;
    std::string returned_value;    
    switch( property_type & XType_Base_Mask )
    {
        case XType_Base_Number:
        case XType_Base_Enum:
        case XType_Base_Bool:           
            // there is no way in xenics API to determine if a property value is long or double, so value is interpreted as double if it has a decimal point, otherwise it is interpreted as long
            if( !( value.find('.') == std::string::npos ) )
            {
                double f = boost::lexical_cast< double >( value );
                error_code = XC_SetPropertyValueF(handle, name.c_str(), f, "");
                double returned_f = 0;
                XC_GetPropertyValueF(handle, name.c_str(), &returned_f);
                mismatched = ( f != returned_f );
                returned_value = boost::lexical_cast< std::string >( returned_f );                      
            }
            else 
            {    
                long n = boost::lexical_cast< long >( value );
                error_code = XC_SetPropertyValueL(handle, name.c_str(), n, "");
                long returned_n = 0;
                XC_GetPropertyValueL(handle, name.c_str(), &returned_n);
                mismatched = ( n != returned_n );
                returned_value = boost::lexical_cast< std::string >( returned_n );        
            }
            break;
        case XType_Base_String:
            error_code = XC_SetPropertyValue(handle, name.c_str(), value.c_str(), "");
            char buf[xenicsPropertyMaxLen];
            XC_GetPropertyValue(handle, name.c_str(), buf, xenicsPropertyMaxLen);
            returned_value = buf;
            mismatched = ( value != returned_value );
            break;
        case XType_Base_Blob:
        default:
            COMMA_THROW( comma::exception, "unknown (or blob) property type for attribute \"" << name << "\"" );
    };
    if( error_code != I_OK ) { COMMA_THROW( comma::exception, "failed to set attribute \"" << name << "\": " << xenics_error_to_string_( error_code ) << " (" << error_code << ")" ); }
    if( mismatched ) { COMMA_THROW( comma::exception, "failed to set " << name << " to " << value << " (the camera returns " << returned_value << ")" ); }
}

static std::string xenics_get_attribute_( XCHANDLE& handle, const std::string& name )
{
    XPropType property_type;
    if( XC_GetPropertyType(handle, name.c_str(), &property_type) != I_OK ) 
    { 
        COMMA_THROW( comma::exception, "failed to get property type for \"" << name << "\" (or " << name << " is unavailable)" ); 
    }
    switch( property_type & XType_Base_Mask )
    {
        case XType_Base_Number:
        {
            // using floating point version for all properties since xenics API does not tell if the property value is long or double (XC_GetPropertyValueL can be used if value is an integer)
            double f = 0;
            XC_GetPropertyValueF(handle, name.c_str(), &f);
            return boost::lexical_cast< std::string >( f );          
        }
        case XType_Base_Enum:
        case XType_Base_Bool:
        {
            long n = 0;
            XC_GetPropertyValueL(handle, name.c_str(), &n);
            return boost::lexical_cast< std::string >( n );
        }
        case XType_Base_String:
        {
            char buf[xenicsPropertyMaxLen];
            XC_GetPropertyValue(handle, name.c_str(), buf, xenicsPropertyMaxLen);
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
        char name[xenicsPropertyMaxLen];
        ErrCode error_code = XC_GetPropertyName(handle, i, &name[0], xenicsPropertyMaxLen);
        if( error_code != I_OK ) { COMMA_THROW( comma::exception, "failed to get property name for i=" << i <<" in xenics_attributes_: " << xenics_error_to_string_( error_code ) << " (" << error_code << ")" ); }     
        std::string value = xenics_get_attribute_( handle, std::string( name ) );
        attributes.insert( std::make_pair( name, value ) ); 
    }
    return attributes;    
}
        
static cv::Mat xenics_to_cvmat_( unsigned long height, unsigned long width, FrameType frame_type, std::vector< word >& frame_buffer )
{
    int opencv_type;
    switch( frame_type )
    {
        case FT_8_BPP_GRAY:
            opencv_type = CV_8UC1;
            break;
        case FT_16_BPP_GRAY:
            opencv_type = CV_16UC1; // native format for Gobi 640 GigE
            break;
        case FT_32_BPP_GRAY:
            opencv_type = CV_32SC1; // this case needs to be verified (opencv does not support CV_32UC1 but the camera uses unsigned 32 bit type)
            break;
        case FT_32_BPP_RGB:
        case FT_32_BPP_BGR:
            opencv_type = CV_8UC3; // this case needs to be verified (3 channels with 8bits each need 24 bits but these types takes 32 bits)
            break;
        case FT_32_BPP_RGBA:
        case FT_32_BPP_BGRA:
            opencv_type = CV_8UC4;
            break;
        default:
            COMMA_THROW( comma::exception, "unknown frame type " << frame_type );
    }
    return cv::Mat(height, width, opencv_type, &frame_buffer[0] );
}

static cv::Mat xenics_temperature_to_cvmat_( unsigned long height, unsigned long width, std::vector< double >& temperature_buffer )
{
    int opencv_type = CV_64FC1;
    return cv::Mat(height, width, opencv_type, &temperature_buffer[0] );
}


class gobi::impl
{
    public:
        impl( const std::string& address, const attributes_type& attributes ) :
            address_( address )
            , closed_( false )
            , thermography_is_enabled_( false )
            , output_temperature_ ( false )
        {
            if( address_.empty() ) { COMMA_THROW( comma::exception, "expected camera address, got empty string" ); }    
            std::string camera_name = "gev://" + address_;
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + xenicsTimeout;             
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                handle_ = XC_OpenCamera( camera_name.c_str() );
                if( XC_IsInitialised( handle_ ) ) break;
                boost::this_thread::sleep( xenicsSleepDuration );
            }
            if( !XC_IsInitialised(handle_) ) { close(); COMMA_THROW( comma::exception, "failed to open gobi camera " + camera_name ); }
            set( attributes );
            height_ = boost::lexical_cast< unsigned long >( XC_GetHeight( handle_ ) );
            width_ = boost::lexical_cast< unsigned long >( XC_GetWidth( handle_ ) );
            frame_type_ = XC_GetFrameType( handle_ );
            total_bytes_per_frame_ = boost::lexical_cast< unsigned long >( XC_GetFrameSize( handle_ ) );
            frame_footer_size_in_bytes_ = boost::lexical_cast< unsigned long >( XC_GetFrameFooterLength( handle_ ) );
            unsigned long frame_footer_size_in_words_ = frame_footer_size_in_bytes_ / 2;
            frame_buffer_.resize( width_ * height_ + frame_footer_size_in_words_ );
            footer_ = ( XPFF_GENERIC* )( &frame_buffer_[0] + width_ * height_ );
        }
        
        void set( const attributes_type& attributes )
        {
            for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
            {
                std::string name = i->first;
                std::string value = i->second;
                xenics_set_attribute_( handle_, name, value );
            }     
        }
        
        ~impl() { close(); }

        void close()
        {
            if( XC_IsCapturing( handle_ ) ) 
            {
                if( XC_StopCapture( handle_ ) != I_OK ) { std::cerr << "could not stop capturing" << std::endl; return; }
            }
            if( XC_IsInitialised( handle_ ) ) XC_CloseCamera( handle_ );
            thermography_is_enabled_ = false;
            closed_ = true;
            //std::cerr << "the camera has been closed" << std::endl;
        }
        
        void enable_thermography( std::string temperature_unit, std::string calibration_file )
        {
            if( thermography_is_enabled_ ) { COMMA_THROW( comma::exception, "thermography already enabled" ); }
            if( !XC_IsInitialised( handle_ ) ) { COMMA_THROW( comma::exception, "cannot enable thermography, since the camera has not been initialised" ); }
            if( XC_LoadCalibration( handle_, calibration_file.c_str(), XLC_StartSoftwareCorrection ) != I_OK )
            { 
                COMMA_THROW( comma::exception, "failed to load calibration file \"" << calibration_file << "\"" ); 
            }
            FilterID fltThermography = 0;
            fltThermography = XC_FLT_Queue( handle_, "Thermography", temperature_unit.c_str() );
            if ( fltThermography > 0 )
            {
                unsigned long max_pixel_value = XC_GetMaxValue( handle_ );
                temperature_from_pixel_value_.resize( max_pixel_value + 1 );
                for( unsigned long i = 0; i < max_pixel_value + 1; ++i )
                {
                    XC_FLT_ADUToTemperature( handle_, fltThermography, i, &temperature_from_pixel_value_[i] );
                }
                temperature_buffer_.resize( width_ * height_ );
                temperature_unit_ = temperature_unit;
                thermography_is_enabled_ = true;
            }
            else
            {
                COMMA_THROW( comma::exception, "could not start thermography filter" );
            }
        }
        
        void disable_thermography() { thermography_is_enabled_ = false; }
        
        void output_conversion( std::string file_name )
        {
            if( !thermography_is_enabled_ ) { COMMA_THROW( comma::exception, "failed to output conversion table, since thermography was not enabled" ); }
            std::ofstream ofs ( file_name.c_str() );
            if( !ofs ) { COMMA_THROW( comma::exception, "failed to output conversion table, unable to create and/or open file: " << file_name ); };
            for( unsigned long i=0; i < temperature_from_pixel_value_.size(); ++i) ofs << i << "," << temperature_from_pixel_value_[i] << std::endl;
        }
        
        std::pair< boost::posix_time::ptime, cv::Mat > read()
        {
            if( !XC_IsCapturing( handle_ ) )
            {
                ErrCode error_code = XC_StartCapture( handle_ );
                if( error_code != I_OK )
                {
                    COMMA_THROW( comma::exception, "failed to start frame capture from xenics camera at " << address_ << ": " << xenics_error_to_string_( error_code ) << "(" << error_code << ")" );
                }
            }
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            ErrCode error_code = XC_GetFrame( handle_, FT_NATIVE, XGF_Blocking | XGF_NoConversion | XGF_FetchPFF, &frame_buffer_[0], total_bytes_per_frame_ );
            if( error_code == I_OK)
            {
                long long time_of_reception_in_microseconds_since_epoch = footer_->tft;
                boost::posix_time::ptime time_of_reception = ptime_( time_of_reception_in_microseconds_since_epoch );
                boost::posix_time::ptime utc = boost::posix_time::microsec_clock::universal_time();
                long time_delay = ( utc - time_of_reception ).total_microseconds();
                if ( time_delay < 0 )
                {
                    COMMA_THROW( comma::exception, "difference between utc and time_of_reception is negative: time of reception = " << time_of_reception << ", utc = " << utc );
                }
                const long max_allowed_time_delay = 1000000l;
                if ( time_delay > max_allowed_time_delay)
                {
                    COMMA_THROW( comma::exception, "difference between utc and time_of_reception is greater than " << max_allowed_time_delay << " microseconds: time of reception = " << time_of_reception << ", utc = " << utc );
                }
                pair.first = time_of_reception;
                if( output_temperature_ )
                {
                    for( unsigned long i = 0; i < temperature_buffer_.size(); ++i ) { temperature_buffer_[i] = temperature_from_pixel_value_[frame_buffer_[i]]; }
                    pair.second = xenics_temperature_to_cvmat_( height_, width_, temperature_buffer_ );
                }
                else
                {
                     pair.second = xenics_to_cvmat_( height_, width_, frame_type_, frame_buffer_ );
                }
            }
            else
            {
                COMMA_THROW( comma::exception, "failed to get a frame from xenics camera at address " << address_ << ": " << xenics_error_to_string_( error_code ) << "(" << error_code << ")" );
            }      
            return pair;
        }                

        const XCHANDLE& handle() const { return handle_; }
        
        XCHANDLE& handle() { return handle_; }

        std::string address() const { return address_; }
        
        bool closed() const { return closed_; }

        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }
        
        std::string temperature_unit() const { return temperature_unit_; }

        static std::vector< XDeviceInformation > list_cameras()
        {
            std::vector< XDeviceInformation > list;
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + xenicsTimeout;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                unsigned int camera_count = 0;
                if( XCD_EnumerateDevices(NULL, &camera_count, XEF_GigEVision) == I_OK && camera_count != 0)
                {
                    list.resize( camera_count );
                    if( XCD_EnumerateDevices(&list[0], &camera_count, XEF_UseCached) == I_OK ) break;
                }
                boost::this_thread::sleep( xenicsSleepDuration );
            }
            return list;
        }
        
        static std::string format_camera_info(const XDeviceInformation& camera_info)
        {
            std::string state = camera_info.state == XDS_Available ? "Available" : camera_info.state == XDS_Busy ? "Busy" : "Unreachable";
            std::stringstream output;
            output << "id=" << camera_info.pid << "," 
                   << "name=\"" << camera_info.name << "\"" << "," 
                   << "serial=\"" << camera_info.serial << "\"" << "," 
                   << "address=\"" << camera_info.address << "\"" << ","
                   << "transport=\"" << camera_info.transport << "\"" << "," 
                   << "state=\"" << state << "\"";
            return output.str();
        }
        
    private:
        XCHANDLE handle_;
        unsigned long height_;
        unsigned long width_;
        FrameType frame_type_;
        unsigned long total_bytes_per_frame_;
        unsigned long frame_footer_size_in_bytes_;
        std::vector< word > frame_buffer_;
        std::vector< double > temperature_from_pixel_value_;
        std::vector< double > temperature_buffer_;
        std::string temperature_unit_;
        std::string address_;
        bool closed_;
        bool thermography_is_enabled_;
        bool output_temperature_;
        XPFF_GENERIC* footer_;
        boost::posix_time::ptime ptime_( long long microseconds_since_epoch )
        {
            return boost::posix_time::time_from_string("1970-01-01 00:00:00.000") + boost::posix_time::microseconds( microseconds_since_epoch );
        }
};

} } // namespace snark{ namespace camera{

namespace snark{ namespace camera{

gobi::gobi( const std::string& address, const gobi::attributes_type& attributes ) : pimpl_( new impl( address, attributes ) ) {}

gobi::~gobi() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > gobi::read() { return pimpl_->read(); }

void gobi::close() { pimpl_->close(); }

bool gobi::closed() const { return pimpl_->closed(); }

std::vector< XDeviceInformation > gobi::list_cameras() { return gobi::impl::list_cameras(); }

std::string gobi::format_camera_info( const XDeviceInformation& camera_info ) { return gobi::impl::format_camera_info( camera_info ); }

std::string gobi::address() const { return pimpl_->address(); }

std::string gobi::temperature_unit() const { return pimpl_->temperature_unit(); }

unsigned long gobi::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

gobi::attributes_type gobi::attributes() const { return xenics_attributes_( pimpl_->handle() ); }

void gobi::set(const gobi::attributes_type& attributes ) { pimpl_->set( attributes ); }

void gobi::enable_thermography( std::string temperature_unit, std::string calibration_file ) { pimpl_->enable_thermography( temperature_unit, calibration_file ); }

void gobi::disable_thermography() { pimpl_->disable_thermography(); }

void gobi::output_conversion( std::string file_name ) { pimpl_->output_conversion( file_name ); }

} } // namespace snark{ namespace camera{
