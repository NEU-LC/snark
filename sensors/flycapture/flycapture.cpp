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
#include <boost/assign.hpp>
#include <boost/bimap.hpp>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include "flycapture.h"

//using namespace FlyCapture2;

// static void PVDECL pv_callback_( tPvFrame *frame );

/*TODO:
* Discard argument is ignored.
* implement a callback solution
*/

namespace snark{ namespace camera{ 

static const unsigned int max_retries = 15;

typedef boost::bimap<FlyCapture2::PixelFormat , std::string> pixel_format_map_t;
static const pixel_format_map_t pixel_format_map = boost::assign::list_of< pixel_format_map_t::relation >
     (FlyCapture2::PIXEL_FORMAT_MONO8, "PIXEL_FORMAT_MONO8")
     (FlyCapture2::PIXEL_FORMAT_411YUV8, "PIXEL_FORMAT_411YUV8")     /**< YUV 4:1:1. */
     (FlyCapture2::PIXEL_FORMAT_422YUV8, "PIXEL_FORMAT_422YUV8")     /**< YUV 4:2:2. */
     (FlyCapture2::PIXEL_FORMAT_444YUV8, "PIXEL_FORMAT_444YUV8")     /**< YUV 4:4:4. */
     (FlyCapture2::PIXEL_FORMAT_RGB8   , "PIXEL_FORMAT_RGB8")        /**< R = G = B = 8 bits. */
     (FlyCapture2::PIXEL_FORMAT_MONO16 , "PIXEL_FORMAT_MONO16")      /**< 16 bits of mono information. */
     (FlyCapture2::PIXEL_FORMAT_RGB16  , "PIXEL_FORMAT_RGB16")       /**< R = G = B = 16 bits. */
     (FlyCapture2::PIXEL_FORMAT_S_MONO16, "PIXEL_FORMAT_S_MONO16")    /**< 16 bits of signed mono information. */
     (FlyCapture2::PIXEL_FORMAT_S_RGB16 , "PIXEL_FORMAT_S_RGB16")     /**< R = G = B = 16 bits signed. */
     (FlyCapture2::PIXEL_FORMAT_RAW8    , "PIXEL_FORMAT_RAW8")        /**< 8 bit raw data output of sensor. */
     (FlyCapture2::PIXEL_FORMAT_RAW16   , "PIXEL_FORMAT_RAW16")       /**< 16 bit raw data output of sensor. */
     (FlyCapture2::PIXEL_FORMAT_MONO12  , "PIXEL_FORMAT_MONO12")      /**< 12 bits of mono information. */
     (FlyCapture2::PIXEL_FORMAT_RAW12   , "PIXEL_FORMAT_RAW12")       /**< 12 bit raw data output of sensor. */
     (FlyCapture2::PIXEL_FORMAT_BGR     , "PIXEL_FORMAT_BGR")         /**< 24 bit BGR. */
     (FlyCapture2::PIXEL_FORMAT_BGRU    , "PIXEL_FORMAT_BGRU")        /**< 32 bit BGRU. */
//     (FlyCapture2::PIXEL_FORMAT_RGB ,"PIXEL_FORMAT_RGB")         /**< 24 bit RGB. same as RGB8*/
     (FlyCapture2::PIXEL_FORMAT_RGBU    , "PIXEL_FORMAT_RGBU")        /**< 32 bit RGBU. */
     (FlyCapture2::PIXEL_FORMAT_BGR16   , "PIXEL_FORMAT_BGR16")       /**< R = G = B = 16 bits. */
     (FlyCapture2::PIXEL_FORMAT_BGRU16  , "PIXEL_FORMAT_BGRU16")      /**< 64 bit BGRU. */
     (FlyCapture2::PIXEL_FORMAT_422YUV8_JPEG, "PIXEL_FORMAT_422YUV8_JPEG")/**< JPEG compressed stream. */
     (FlyCapture2::NUM_PIXEL_FORMATS        , "NUM_PIXEL_FORMATS")        /**< Number of pixel formats. */
     (FlyCapture2::UNSPECIFIED_PIXEL_FORMAT, "UNSPECIFIED_PIXEL_FORMAT");

typedef boost::bimap<FlyCapture2::PropertyType ,std::string > property_map_t;
property_map_t property_map = boost::assign::list_of<property_map_t::relation>
    (FlyCapture2::BRIGHTNESS,        "brightness")
    (FlyCapture2::AUTO_EXPOSURE,     "auto_exposure")
    (FlyCapture2::SHARPNESS,         "sharpness")
    (FlyCapture2::WHITE_BALANCE,     "white_balance")
    (FlyCapture2::HUE,               "hue")
    (FlyCapture2::SATURATION,        "saturation")
    (FlyCapture2::GAMMA,             "gamma")
    (FlyCapture2::IRIS,              "iris")
    (FlyCapture2::FOCUS,             "focus")
    (FlyCapture2::ZOOM,              "zoom")
    (FlyCapture2::PAN,               "pan")
    (FlyCapture2::TILT,              "tilt")
    (FlyCapture2::SHUTTER,           "shutter")
    (FlyCapture2::GAIN,              "gain")
    //(FlyCapture2::TRIGGER_MODE,      "trigger_mode")  //Not supported properly through PropertyType
    //(FlyCapture2::TRIGGER_DELAY,     "trigger_delay") //There is a separate function set for trigger modes 
    (FlyCapture2::FRAME_RATE,        "frame_rate")
    (FlyCapture2::TEMPERATURE,       "temperature");
    
//List for attributes that exist inside structs
static const std::string structAttributes[] = {"maxWidth", "maxHeight", "offsetHStepSize" , "offsetVStepSize",
  "offsetX" , "offsetY" , "width" , "height" , "PixelFormat" ,"trigger_on", "trigger_mode" , "trigger_parameter",
  "trigger_polarity" , "trigger_source"};     
    
static std::string flycapture_get_attribute_( FlyCapture2::GigECamera& handle, const std::string& key )
{
    FlyCapture2::Error error;
    FlyCapture2::GigEImageSettings image_settings;
    FlyCapture2::GigEImageSettingsInfo image_settings_info;
    FlyCapture2::TriggerMode trigger_mode;

    error = handle.GetGigEImageSettings(&image_settings);
    error = handle.GetGigEImageSettingsInfo(&image_settings_info);
    error = handle.GetTriggerMode(&trigger_mode); 
    
    //ImageSettingsInfo struct
    /***/if ( key == "maxWidth" )        return boost::to_string( image_settings_info.maxWidth );
    else if ( key == "maxHeight" )       return boost::to_string( image_settings_info.maxHeight );
    else if ( key == "offsetHStepSize" ) return boost::to_string( image_settings_info.offsetHStepSize );
    else if ( key == "offsetVStepSize" ) return boost::to_string( image_settings_info.offsetVStepSize );
     
    //ImageSettings struct
    else if ( key == "offsetX" )     return boost::to_string( image_settings.offsetX );
    else if ( key == "offsetY" )     return boost::to_string( image_settings.offsetY );
    else if ( key == "width" )       return boost::to_string( image_settings.width );
    else if ( key == "height" )      return boost::to_string( image_settings.height );
    else if ( key == "PixelFormat" ) return pixel_format_map.left.at( image_settings.pixelFormat );
    
    else if ( key == "trigger_on" )        return boost::to_string(trigger_mode.onOff);
    else if ( key == "trigger_polarity" )  return boost::to_string(trigger_mode.polarity);
    else if ( key == "trigger_source" )    return boost::to_string(trigger_mode.source);
    else if ( key == "trigger_mode" )      return boost::to_string(trigger_mode.mode);
    else if ( key == "trigger_parameter" ) return boost::to_string(trigger_mode.parameter);
    //Check the property list
    else{
        if( property_map.right.find(key) != property_map.right.end() )
        {
            FlyCapture2::Property cam_prop;
            FlyCapture2::PropertyInfo cam_prop_info;
            
            cam_prop.type = property_map.right.at( key );
            cam_prop_info.type = property_map.right.at( key );
            
            handle.GetProperty( &cam_prop );
            handle.GetPropertyInfo( &cam_prop_info );
            
            if( !cam_prop.present ) return "N/A";    //If property is not present, it is unsupported for this camera
            if( cam_prop.autoManualMode ) return "auto";
            if( cam_prop.type == FlyCapture2::WHITE_BALANCE ) //White balance has two values, so it is a special case
                return boost::to_string( cam_prop.valueA ) + "," + boost::to_string( cam_prop.valueB );
    
            return boost::to_string( cam_prop.absControl ? cam_prop.absValue : cam_prop.valueA );
  
        } else 
        {
          std::cerr << "Property: " << key << " not found!" << std::endl;
        }
    }
   return "Not Found"; 
}

static void flycapture_set_attribute_( FlyCapture2::GigECamera& handle, const std::string& key, const std::string& value )
{
    FlyCapture2::Error error;
    FlyCapture2::GigEConfig cam_config;
    FlyCapture2::GigEImageSettings image_settings;
    FlyCapture2::GigEImageSettingsInfo image_settings_info;
    FlyCapture2::TriggerMode trigger_mode;

    error = handle.GetGigEImageSettings(&image_settings);
    if(error != FlyCapture2::PGRERROR_OK)
        std::cerr << "Error getting attributes from camera." << std::endl;

    error = handle.GetGigEImageSettingsInfo(&image_settings_info);            
    if(error != FlyCapture2::PGRERROR_OK)
        std::cerr << "Error getting attributes from camera." << std::endl;

    error = handle.GetTriggerMode(&trigger_mode);            
    if(error != FlyCapture2::PGRERROR_OK)
        std::cerr << "Error getting attributes from camera." << std::endl;
    
    //ImageSettingsInfo struct
    /**/ if ( key == "offsetHStepSize" ) image_settings_info.offsetHStepSize = boost::lexical_cast<int>(value);
    else if ( key == "offsetVStepSize" ) image_settings_info.offsetVStepSize = boost::lexical_cast<int>(value);
     
    //ImageSettings struct
    else if ( key == "offsetX" )     image_settings.offsetX = boost::lexical_cast<int>(value);
    else if ( key == "offsetY" )     image_settings.offsetY = boost::lexical_cast<int>(value);
    else if ( key == "width" )       
    {
        if(( boost::lexical_cast<uint>( value ) > image_settings_info.maxWidth ) |
            ( boost::lexical_cast<uint>( value ) < 0 ))
            std::cerr << "Error: width out of bounds" << std::endl;
        else
          image_settings.width = boost::lexical_cast<int>( value );
    }
    else if ( key == "height" )
    {
        if(( boost::lexical_cast<uint>( value ) > image_settings_info.maxHeight ) |
            ( boost::lexical_cast<uint>( value ) < 0 ))
            std::cerr << "Error: height out of bounds" << std::endl;
        else
          image_settings.height = boost::lexical_cast<int>( value );
    }    
    else if ( key == "PixelFormat" ) 
    {
        if( pixel_format_map.right.find(value) != pixel_format_map.right.end() )
              image_settings.pixelFormat = pixel_format_map.right.at( value );
        else
              std::cerr << "Error, invalid pixel format." << std::endl;
    } 
    else if ( key == "trigger_on" ) 
    {
      if ( value == "true" )
          trigger_mode.onOff = true;
      else if ( value == "false" )
          trigger_mode.onOff = false;
      else
          std::cerr << "Error, invalid trigger_on setting. Please use true/false" << std::endl;
    }  
    else if ( key == "trigger_polarity" ) //TODO confirm integer values
    {
       if ( value == "high" )
          trigger_mode.polarity = 1;
      else if ( value == "low" )
          trigger_mode.polarity = 0;
      else
          std::cerr << "Error, invalid trigger_polarity setting. Please use high/low" << std::endl;
    }        
    else if ( key == "trigger_source" ) // 0-3 are GPIO, 4 = none
    {
        /**/ if( value == "GPIO0" ) trigger_mode.source = 0; 
        else if( value == "GPIO1" ) trigger_mode.source = 1;
        else if( value == "GPIO2" ) trigger_mode.source = 2;
        else if( value == "GPIO3" ) trigger_mode.source = 3;
        else if( value == "none"  ) trigger_mode.source = 4;
        else if( value == "software" ) trigger_mode.source = 7;
        else std::cerr << "Error unknown trigger source. please use 'GPIO[0-3]', 'software' or 'none'" << std::endl;
    }
    else if ( key == "trigger_mode" )      trigger_mode.mode = boost::lexical_cast<int>(value);
    else if ( key == "trigger_parameter" ) trigger_mode.parameter = boost::lexical_cast<int>(value);

    //Check the property list
    else{
        if( property_map.right.find( key ) != property_map.right.end() )
        {
            FlyCapture2::Property cam_prop;
            FlyCapture2::PropertyInfo cam_prop_info;
            
            cam_prop.type = property_map.right.at( key );
            cam_prop_info.type = property_map.right.at( key );
            
            handle.GetProperty( &cam_prop );
            handle.GetPropertyInfo( &cam_prop_info );
            
            if( !cam_prop.present ) return;    //If property is not present, it is unsupported for this camera
            if( value == "auto" ) 
                cam_prop.autoManualMode = true;
            else
            {
                cam_prop.autoManualMode = false;
                if( cam_prop.type == FlyCapture2::WHITE_BALANCE ) //White balance has two values, so it is a special case
                {
                    std::vector< std::string > v = comma::split( value, "," );
                    if(v.size() != 2)
                    {
                        std::cerr << "Error: White Balance must be in the format of 'red,blue' or 'auto' where red and blue are integers [0,1023]" << std::endl;
                    } 
                    else   
                    {
                       cam_prop.valueA = boost::lexical_cast<uint>( v[0] );
                       cam_prop.valueB = boost::lexical_cast<uint>( v[1] );
                    }
                }
                else 
                {  
                    if( value.find( "." ) != value.npos )
                       {
                           cam_prop.absControl = true;
                           cam_prop.absValue = boost::lexical_cast<float>( value );             
                       }
                       else
                       {
                         cam_prop.absControl = false;
                           cam_prop.valueA = boost::lexical_cast<uint>( value );
                       }
                }
            }
            error = handle.SetProperty( &cam_prop );
            if(error != FlyCapture2::PGRERROR_OK)
                std::cerr << "Error setting attributes: " << error.GetDescription() << std::endl;
        } else 
        {
          std::cerr << "Property: " << key << " not found!" << std::endl;
        }
    }
    //Handle errors here, the SDK should do out of bounds checking (test width > 1920 for example)
    
    error = handle.SetGigEImageSettings( &image_settings );
    if(error != FlyCapture2::PGRERROR_OK)
        std::cerr << "Error setting attributes." << std::endl;
    
    error = handle.SetTriggerMode( &trigger_mode );
    if(error != FlyCapture2::PGRERROR_OK)
        std::cerr << "Error setting attributes." << std::endl;

    return;
}


flycapture::attributes_type flycapture_attributes_( FlyCapture2::GigECamera& handle )
{
    flycapture::attributes_type attributes;
    for ( uint i = 0; i < ( sizeof( structAttributes ) / sizeof( std::string ) ); i++ ) 
    {
       attributes.insert( std::make_pair(structAttributes[i].c_str(), flycapture_get_attribute_( handle, structAttributes[i].c_str() ) ) );
    }
     
    //There is an iterable list for the remaining attributes
    for( property_map_t::const_iterator i = property_map.begin(); i != property_map.end(); ++i )
    {
       attributes.insert( std::make_pair( i->right , flycapture_get_attribute_( handle,i->right ) ) ); 
    }
    return attributes;
}

unsigned int flycapture_bits_per_pixel_( const FlyCapture2::PixelFormat pixel_format_ )
{
    switch( pixel_format_ )
    {
        case FlyCapture2::PIXEL_FORMAT_MONO8:
        case FlyCapture2::PIXEL_FORMAT_RAW8:
            return 8;
            break;
        case FlyCapture2::PIXEL_FORMAT_RAW16:
        case FlyCapture2::PIXEL_FORMAT_MONO16:
            return 16;
        case FlyCapture2::PIXEL_FORMAT_RGB:
        case FlyCapture2::PIXEL_FORMAT_BGR:
            return 24;
        case FlyCapture2::PIXEL_FORMAT_RGBU:
        case FlyCapture2::PIXEL_FORMAT_BGRU:
            return 32;
        case FlyCapture2::PIXEL_FORMAT_RGB16:
            return 48;
        case FlyCapture2::PIXEL_FORMAT_411YUV8:
        case FlyCapture2::PIXEL_FORMAT_422YUV8:
        case FlyCapture2::PIXEL_FORMAT_444YUV8:
            COMMA_THROW( comma::exception, "unsupported format " << pixel_format_ );
        default:
            COMMA_THROW( comma::exception, "unknown format " << pixel_format_ );  
        return 0;
    }
}

cv::Mat flycapture_image_as_cvmat_( const FlyCapture2::Image& frame )
{
    int type;
    switch( flycapture_bits_per_pixel_(frame.GetPixelFormat() ) ) 
    {
        case 8:
            type = CV_8UC1;
            break;
        case 16:
            type = CV_16UC1;
            break;
        case 24:
            type = CV_8UC3;
            break;
        case 32:
            type = CV_8UC4;
            break;
        case 48:
            type = CV_16UC3;
            break;
        default:
            COMMA_THROW( comma::exception, "unknown format " << frame.GetPixelFormat()  );
    };

    return cv::Mat( frame.GetRows(), frame.GetCols(), type, frame.GetData() );
}

bool flycapture_collect_frame_(cv::Mat & image, FlyCapture2::GigECamera& handle, bool & started)
{
    FlyCapture2::Error result;
    FlyCapture2::Image frame_;
    FlyCapture2::Image raw_image;
    
    result = handle.RetrieveBuffer(&raw_image);
    frame_.DeepCopy(&raw_image);
    raw_image.ReleaseBuffer();
    
    if( result == FlyCapture2::PGRERROR_OK ) 
    {
      cv::Mat cvImage = flycapture_image_as_cvmat_( frame_ );
      cvImage.copyTo(image);
      return true;
    } 
    else if( result == FlyCapture2::PGRERROR_IMAGE_CONSISTENCY_ERROR )
    {   //report damaged frame and try again
        FlyCapture2::CameraInfo camera_info;
        handle.GetCameraInfo(&camera_info);
        std::cerr << "Error line " << __LINE__ << ": " << result.GetDescription() << "from camera: " << camera_info.serialNumber << " Retrying..." << std::endl;
        return false; 
    } 
    else if ( ( result == FlyCapture2::PGRERROR_ISOCH_START_FAILED ) //These are errors that result in a retry
      | ( result == FlyCapture2::PGRERROR_ISOCH_ALREADY_STARTED )
      | ( result == FlyCapture2::PGRERROR_ISOCH_NOT_STARTED ) )
    {
        std::cerr << "Error: " << result.GetDescription() << " Restarting camera." << std::endl;
        handle.StopCapture();
        started = false;
        return false;
    }
    else
    {  //Fatal
      FlyCapture2::CameraInfo camera_info;
      handle.GetCameraInfo(&camera_info);
      COMMA_THROW( comma::exception, "got frame with invalid status on camera " << camera_info.serialNumber << ": " << result.GetType() << ": " << result.GetDescription() );
    }

}

class flycapture::impl
{
    //Note, for Point Grey, the serial number is used as ID
    public:
        impl( unsigned int id, const attributes_type& attributes, unsigned int id_right ) :
            started_( false ),
            timeOut_( 1000 )
        {
            initialize_();
            
            stereo = false;
            if(id_right != 0)
                {stereo = true; }
            
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            unsigned int size = 0;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                const std::vector< FlyCapture2::CameraInfo >& list = list_cameras();
                size = list.size();

                for( unsigned int i = 0; i < list.size(); ++i ) // look for a point grey camera that matches the serial number
                {
                    if(list[i].interfaceType == FlyCapture2::INTERFACE_GIGE && (id == 0))
                        id_ = list[i].serialNumber;
                    if(list[i].interfaceType == FlyCapture2::INTERFACE_GIGE && ( id == list[i].serialNumber ))
                        id_ = list[i].serialNumber;
                    if(list[i].interfaceType == FlyCapture2::INTERFACE_GIGE && ( id_right == list[i].serialNumber ))
                        id_right_ = list[i].serialNumber;
                }
                if( id_ && !stereo ) { break; }
                if(stereo && id_ && id_right_ ) { break; }
             }
             if( !id_ ) { COMMA_THROW( comma::exception, "timeout; camera not found" ); }
             if(stereo && !id_right_ ) { COMMA_THROW( comma::exception, "timeout; stereo right camera not found" ); }

             if( id == 0 && size > 1)
             {
                const std::vector< FlyCapture2::CameraInfo >& list = list_cameras();
                std::stringstream stream;
                for( std::size_t i = 0; i < list.size(); ++i ) // todo: serialize properly with name-value
                {
                     stream << "serial=\"" << list[i].serialNumber << "\"," << "model=\"" << list[i].modelName << "\"" << std::endl;
                }
                COMMA_THROW( comma::exception, "no id provided and multiple cameras found: would pick up a random one\n\tavailable cameras:\n" << stream.str() );
            }
            now = boost::posix_time::microsec_clock::universal_time();
            end = now + timeout;
   
            //Get Point grey unique id (guid) from serial number. guid does not exist in CameraInfo, and so it does not appear in the camera list
            FlyCapture2::BusManager bus_manager;
            bus_manager.GetCameraFromSerialNumber( *id_ , &guid );
     
            FlyCapture2::Error result = handle_.Connect( &guid );
            for( ; ( result != FlyCapture2::PGRERROR_OK ) && ( now < end ); now = boost::posix_time::microsec_clock::universal_time() )
            {
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
                result = handle_.Connect(&guid);
            }
            if (result != FlyCapture2::PGRERROR_OK){close(); COMMA_THROW( comma::exception, "failed to open point grey camera: " << id << ", Reason: " << result.GetDescription() );}
            
            for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
            {
                flycapture_set_attribute_( handle_, i->first, i->second );
            }   
            
            end = now + timeout;
            if( stereo )
            {
                bus_manager.GetCameraFromSerialNumber(*id_right_, &guid_right);
                FlyCapture2::Error result = handle_right_.Connect(&guid_right);

                for( ; ( result != FlyCapture2::PGRERROR_OK ) && ( now < end ); now = boost::posix_time::microsec_clock::universal_time() )
                {
                    boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
                    result = handle_right_.Connect(&guid_right);
                }
                if (result != FlyCapture2::PGRERROR_OK){close(); COMMA_THROW( comma::exception, "failed to open point grey camera: " << id_right << ", Reason: " << result.GetDescription() );}
            }
            
            FlyCapture2::CameraInfo camera_info;
            if(stereo)
            {
                handle_right_.GetCameraInfo( &camera_info );
            }
            handle_.GetCameraInfo( &camera_info );
 
            
            if(stereo)
            { // Trigger mode is used to synchronize shutters between the cameras
               flycapture_set_attribute_( handle_, "trigger_on", "false" );     
               flycapture_set_attribute_( handle_right_, "trigger_mode", "0" );
               flycapture_set_attribute_( handle_right_, "trigger_on", "true" );
               flycapture_set_attribute_( handle_right_, "trigger_source", "GPIO0" );
	       
	       //An extended packet delay is needed to stop packet collision
	       //TODO Maybe add this to set_attr so that it can be user customisable
	       FlyCapture2::GigEProperty packetDelay;
	       packetDelay.propType = FlyCapture2::PACKET_DELAY;
	       handle_.GetGigEProperty(&packetDelay);
	       packetDelay.value = 2000;
	       handle_.SetGigEProperty(&packetDelay);
	       handle_right_.GetGigEProperty(&packetDelay);
	       packetDelay.value = 2100;
	       handle_right_.SetGigEProperty(&packetDelay);
            }

            uint width;
            uint height;
            FlyCapture2::PixelFormat pixel_format;
            width = boost::lexical_cast<uint>( flycapture_get_attribute_( handle_, "width" ) );
            height = boost::lexical_cast<uint>( flycapture_get_attribute_( handle_, "height" ) );
            pixel_format = pixel_format_map.right.at( flycapture_get_attribute_( handle_,"PixelFormat" ) );

            total_bytes_per_frame_ = ( stereo ? 2 : 0 ) * width * height
                                          * flycapture_bits_per_pixel_( pixel_format ) / 8;

        }

        ~impl() { close(); }

        void close()
        {
            id_.reset();
            if( !handle_.IsConnected() ) { return; }
            handle_.StopCapture();
            handle_.Disconnect();
            if( stereo )
            {
              id_right_.reset();
              if( !handle_right_.IsConnected() ) { return; }
              handle_right_.StopCapture();
              handle_right_.Disconnect();
            }
            //std::cerr << "the camera has been closed" << std::endl;
        }

        std::pair< boost::posix_time::ptime, cv::Mat > read()
        {
            cv::Mat image_;
            cv::Mat image_right_;
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            bool success = false;
            unsigned int retries = 0;

            while( !success && retries < max_retries )
            {
                FlyCapture2::Error result;
                
                if( !started_ )
                    {result = handle_.StartCapture();}
                // error is not checked as sometimes the camera
                // will start correctly but return an error
                started_ = true;

                if( stereo )
                {
                    if(!started_right_)
                        result =  handle_right_.StartCapture();
                    started_right_ = true;
                
                } 
                success = flycapture_collect_frame_( image_, handle_ , started_ );
                pair.first = boost::posix_time::microsec_clock::universal_time();
                if( success )
                {
                    if( !stereo )
                    {
                        pair.second = image_;
                    } 
                    else 
                    {
                        success = flycapture_collect_frame_( image_right_ , handle_right_ , started_right_ );
                        if ( success )
                        {
                            cv::hconcat( image_ , image_right_ , pair.second );
                        } 
                    }
                }
                retries++;
             }
             if( success ) { return pair; }
             COMMA_THROW( comma::exception, "got lots of missing frames or timeouts" << std::endl << std::endl << "it is likely that MTU size on your machine is less than packet size" << std::endl << "check PacketSize attribute (flycapture-cat --list-attributes)" << std::endl << "set packet size (e.g. flycapture-cat --set=PacketSize=1500)" << std::endl << "or increase MTU size on your machine" );
        }
        
        const FlyCapture2::GigECamera& handle() const { return handle_; }
 
        FlyCapture2::GigECamera& handle() { return handle_; }
        
        const FlyCapture2::GigECamera& handle_right() const { return handle_right_; }
 
        FlyCapture2::GigECamera& handle_right() { return handle_right_; }

        unsigned int id() const { return *id_; }
        
        unsigned int id_right() const { return *id_right_; }

        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }

        static std::vector< FlyCapture2::CameraInfo > list_cameras()
        {
            initialize_();
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            std::vector< FlyCapture2::CameraInfo > list;
    
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                FlyCapture2::BusManager bus_manager;
                unsigned int num_cameras;
                FlyCapture2::Error error;
                error = bus_manager.GetNumOfCameras( &num_cameras );
                if ( error != FlyCapture2::PGRERROR_OK ){
                    COMMA_THROW( comma::exception, "Cannot find point grey cameras");
                }

                FlyCapture2::CameraInfo cam_info[num_cameras];
		//BUG DiscoverGigECameras will sometimes crash if there are devices on a different subnet
                error = FlyCapture2::BusManager::DiscoverGigECameras( cam_info, &num_cameras );
                if ( error != FlyCapture2::PGRERROR_OK ){
                    COMMA_THROW( comma::exception, "Cannot discover point grey cameras" );
                }
                
                //If array is not empty, convert to list and exit
                if( num_cameras > 0 ) {
                    std::copy( &cam_info[0] , &cam_info[num_cameras] , std::back_inserter( list ) );
                    break;
                }
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
            }
            return list;
        }
        
    private:
        friend class flycapture::callback::impl;
        FlyCapture2::GigECamera handle_;
        FlyCapture2::GigECamera handle_right_;
        std::vector< char > buffer_;
        boost::optional< unsigned int > id_;
        boost::optional< unsigned int > id_right_;
        FlyCapture2::PGRGuid guid;
        FlyCapture2::PGRGuid guid_right;
        bool stereo;
        unsigned long total_bytes_per_frame_;
        bool started_;
        bool started_right_;
        unsigned int timeOut_; // milliseconds
        static void initialize_() // quick and dirty
        {
        }
};

class flycapture::callback::impl
{
    public:
        typedef boost::function< void ( const std::pair< boost::posix_time::ptime, cv::Mat >& ) > OnFrame;
        
        impl( flycapture& flycapture, OnFrame on_frame )
            : on_frame( on_frame )
         /*   , handle( flycapture.pimpl_->handle() )
            , frame( flycapture.pimpl_->frame_ )
         */   , good( true )
            , is_shutdown( false )
        {
//             tPvErr result;
//             PvCaptureQueueClear( handle );
//             frame.Context[0] = this;
//             result = PvCaptureStart( handle );
//             if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to start capturing on camera " << flycapture.pimpl_->id() << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
//             result = PvCaptureQueueFrame( handle, &frame, pv_callback_ );
//             if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to set capture queue frame on camera " << flycapture.pimpl_->id() << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
//             result = PvCommandRun( handle, "AcquisitionStart" );
//             if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to start acquisition on camera " << flycapture.pimpl_->id() << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
        }

        ~impl()
        {
            is_shutdown = true;
//             PvCommandRun( handle, "Acquisitionstop" );
//             PvCaptureQueueClear( handle );
//             PvCaptureEnd( handle );
//             handle.StopCapture();
        }

        OnFrame on_frame;
//         tPvHandle& handle;
//         tPvFrame& frame;
        bool good;
        bool is_shutdown;
};

} } // namespace snark{ namespace camera{

// static void PVDECL pv_callback_( tPvFrame *frame )
// {
//     snark::camera::flycapture::callback::impl* c = reinterpret_cast< snark::camera::flycapture::callback::impl* >( frame->Context[0] );
//     if( c->is_shutdown ) { return; }
//     std::pair< boost::posix_time::ptime, cv::Mat > m( boost::posix_time::microsec_clock::universal_time(), cv::Mat() );
//     if( frame ) { m.second = snark::camera::pv_as_cvmat_( *frame ); }
//     c->on_frame( m );
//     tPvErr result = PvCaptureQueueFrame( c->handle, &c->frame, pv_callback_ );
//     if( result != ePvErrSuccess ) { c->good = false; }
// }

namespace snark{ namespace camera{

flycapture::flycapture( unsigned int id, const flycapture::attributes_type& attributes, unsigned int id_right ) : pimpl_( new impl( id, attributes, id_right ) ) {}

flycapture::~flycapture() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > flycapture::read() { return pimpl_->read(); }

void flycapture::close() { pimpl_->close(); }

std::vector< FlyCapture2::CameraInfo > flycapture::list_cameras() { return flycapture::impl::list_cameras(); }

unsigned int flycapture::id() const { return pimpl_->id(); }

unsigned int flycapture::id_right() const { return pimpl_->id_right(); }

unsigned long flycapture::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

flycapture::attributes_type flycapture::attributes() const { return flycapture_attributes_( pimpl_->handle() ); }

flycapture::callback::callback( flycapture& flycapture, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame )
    : pimpl_( new callback::impl( flycapture, on_frame ) )
{
}

flycapture::callback::~callback() { delete pimpl_; }

bool flycapture::callback::good() const { return pimpl_->good; }

} }// namespace snark{ namespace camera{ 
