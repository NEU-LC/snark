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
#include "flycapture.h"

using namespace FlyCapture2;

// static void PVDECL pv_callback_( tPvFrame *frame );

/*TODO:
* Attributes cannot be set, this is currently done through the flycap program provided by point-grey.
* Discard argument is ignored.
* implement a callback solution
*/
namespace snark{ namespace camera{ 

static const unsigned int timeOutFactor = 3;
static const unsigned int maxRetries = 15;
typedef boost::bimap<PixelFormat ,const char*> pixelFormatMap_t;
pixelFormatMap_t pixelFormatMap;
typedef boost::bimap<PropertyType ,std::string > propertyMap_t;
propertyMap_t propertyMap;

/*Create a map of the ENUM PixelFormat so that it can be matched to strings given via command line*/
static void pg_create_PixelFormat_map_(void)
{
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_MONO8, "PIXEL_FORMAT_MONO8"));       /**< 8 bits of mono information. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_411YUV8, "PIXEL_FORMAT_411YUV8"));     /**< YUV 4:1:1. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_422YUV8, "PIXEL_FORMAT_422YUV8"));     /**< YUV 4:2:2. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_444YUV8, "PIXEL_FORMAT_444YUV8"));     /**< YUV 4:4:4. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_RGB8   , "PIXEL_FORMAT_RGB8"));        /**< R = G = B = 8 bits. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_MONO16 , "PIXEL_FORMAT_MONO16"));      /**< 16 bits of mono information. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_RGB16  , "PIXEL_FORMAT_RGB16"));       /**< R = G = B = 16 bits. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_S_MONO16, "PIXEL_FORMAT_S_MONO16"));    /**< 16 bits of signed mono information. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_S_RGB16 , "PIXEL_FORMAT_S_RGB16"));     /**< R = G = B = 16 bits signed. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_RAW8    , "PIXEL_FORMAT_RAW8"));        /**< 8 bit raw data output of sensor. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_RAW16   , "PIXEL_FORMAT_RAW16"));       /**< 16 bit raw data output of sensor. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_MONO12  , "PIXEL_FORMAT_MONO12"));      /**< 12 bits of mono information. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_RAW12   , "PIXEL_FORMAT_RAW12"));       /**< 12 bit raw data output of sensor. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_BGR     , "PIXEL_FORMAT_BGR"));         /**< 24 bit BGR. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_BGRU    , "PIXEL_FORMAT_BGRU"));        /**< 32 bit BGRU. */
 //    pixelFormatMap.insert(pixelFormatMap_t::value_type(case PIXEL_FORMAT_RGB ,"PIXEL_FORMAT_RGB"));         /**< 24 bit RGB. same as RGB8*/
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_RGBU    , "PIXEL_FORMAT_RGBU"));        /**< 32 bit RGBU. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_BGR16   , "PIXEL_FORMAT_BGR16"));       /**< R = G = B = 16 bits. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_BGRU16  , "PIXEL_FORMAT_BGRU16"));      /**< 64 bit BGRU. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(PIXEL_FORMAT_422YUV8_JPEG, "PIXEL_FORMAT_422YUV8_JPEG"));/**< JPEG compressed stream. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type(NUM_PIXEL_FORMATS        , "NUM_PIXEL_FORMATS"));        /**< Number of pixel formats. */
     pixelFormatMap.insert(pixelFormatMap_t::value_type( UNSPECIFIED_PIXEL_FORMAT, "UNSPECIFIED_PIXEL_FORMAT"));
    return;
}

/*Create a map of the ENUM PropertyType so that it can be matched to strings given via command line*/
static void pg_create_property_map_(void){
    propertyMap.insert(propertyMap_t::value_type(BRIGHTNESS,        "brightness"));
    propertyMap.insert(propertyMap_t::value_type(AUTO_EXPOSURE,     "auto_exposure"));
    propertyMap.insert(propertyMap_t::value_type(SHARPNESS,         "sharpness"));
    propertyMap.insert(propertyMap_t::value_type(WHITE_BALANCE,     "white_balance"));
    propertyMap.insert(propertyMap_t::value_type(HUE,               "hue"));
    propertyMap.insert(propertyMap_t::value_type(SATURATION,        "saturation"));
    propertyMap.insert(propertyMap_t::value_type(GAMMA,             "gamma"));
    propertyMap.insert(propertyMap_t::value_type(IRIS,              "iris"));
    propertyMap.insert(propertyMap_t::value_type(FOCUS,             "focus"));
    propertyMap.insert(propertyMap_t::value_type(ZOOM,              "zoom"));
    propertyMap.insert(propertyMap_t::value_type(PAN,               "pan"));
    propertyMap.insert(propertyMap_t::value_type(TILT,              "tilt"));
    propertyMap.insert(propertyMap_t::value_type(SHUTTER,           "shutter"));
    propertyMap.insert(propertyMap_t::value_type(GAIN,              "gain"));
    propertyMap.insert(propertyMap_t::value_type(TRIGGER_MODE,      "trigger_mode"));
    propertyMap.insert(propertyMap_t::value_type(TRIGGER_DELAY,     "trigger_delay"));
    propertyMap.insert(propertyMap_t::value_type(FRAME_RATE,        "frame_rate"));
    propertyMap.insert(propertyMap_t::value_type(TEMPERATURE,       "temperature"));
    return;
}

static std::string pg_get_attribute_(GigECamera& handle, const std::string& key)
{
    //Get all the structs
    Error error;
    CameraInfo camInfo;
    GigEImageSettings imageSettings;
    GigEImageSettingsInfo imageSettingsInfo;    

    error = handle.GetCameraInfo(&camInfo);
    error = handle.GetGigEImageSettings(&imageSettings);
    error = handle.GetGigEImageSettingsInfo(&imageSettingsInfo);            
    //switch on string
    
    //ImageSettingsInfo struct
    /*****/if (0 == key.compare("maxWidth")){    
       return boost::to_string(imageSettingsInfo.maxWidth);
    }
    else if (0 == key.compare("maxHeight")){
        return boost::to_string(imageSettingsInfo.maxHeight);
    }
    else if (0 == key.compare("offsetHStepSize")){
        return boost::to_string(imageSettingsInfo.offsetHStepSize);
    }
    else if (0 == key.compare("offsetVStepSize")){
        return boost::to_string(imageSettingsInfo.offsetVStepSize);
    } 
    //ImageSettings struct
    else if (0 == key.compare("offsetX")){
        return boost::to_string(imageSettings.offsetX);
    }
    else if (0 == key.compare("offsetY")){
        return boost::to_string(imageSettings.offsetY);
    }
    else if (0 == key.compare("width")){
        return boost::to_string(imageSettings.width);
    }
    else if (0 == key.compare("height")){
        return boost::to_string(imageSettings.height);
    }
    else if (0 == key.compare("PixelFormat")){
        return pixelFormatMap.left.at(imageSettings.pixelFormat);
    }  
    // Finally check the property list
    else{
        if(propertyMap.right.find(key) != propertyMap.right.end()){
            Property camProp;
            PropertyInfo camPropInfo;
            
            //Lookup the given property
            camProp.type = propertyMap.right.at(key);
            camPropInfo.type = propertyMap.right.at(key);
            
            //Fetch the property and its info
            handle.GetProperty(&camProp);
            handle.GetPropertyInfo(&camPropInfo);
            
            if(!camProp.present) return "N/A";    //If property is not present, it is unsupported for this camera
            if(camProp.autoManualMode) return "auto";
            if(camProp.type == WHITE_BALANCE){ //White balance has two values, so it is a special case
              return (boost::to_string(camProp.valueA) + "," + boost::to_string(camProp.valueB));
            }
            if(camProp.absControl){ //The value is a floating point number
              return boost::to_string(camProp.absValue);
            } else{ //value is an int
              return boost::to_string(camProp.valueA);
            }
        } else {
          std::cerr << "Property: " << key << " not found!" << std::endl;
        }
    }
   return "Not Found"; 
}

static void pg_set_attribute_( GigECamera& handle, const std::string& key, const std::string& value )
{
    CameraInfo CamInfo;
    GigEConfig CamConfig;
    GigEImageSettings ImageSettings;
    
    return;
}


flycapture::attributes_type pg_attributes_( GigECamera& handle )
{
     flycapture::attributes_type attributes;
     //Some attributes exist inside a struct
     attributes.insert( std::make_pair("maxWidth" , pg_get_attribute_( handle, "maxWidth" ) ) );
     attributes.insert( std::make_pair("maxHeight" , pg_get_attribute_( handle, "maxHeight" ) ) );
     attributes.insert( std::make_pair("offsetHStepSize" , pg_get_attribute_( handle, "offsetHStepSize" ) ) );
     attributes.insert( std::make_pair("offsetVStepSize" , pg_get_attribute_( handle, "offsetVStepSize" ) ) );
     attributes.insert( std::make_pair("offsetX" , pg_get_attribute_( handle, "offsetX" ) ) );
     attributes.insert( std::make_pair("offsetY" , pg_get_attribute_( handle, "offsetY" ) ) );
     attributes.insert( std::make_pair("width" , pg_get_attribute_( handle, "width" ) ) );
     attributes.insert( std::make_pair("height" , pg_get_attribute_( handle, "height" ) ) );
     attributes.insert( std::make_pair("PixelFormat" , pg_get_attribute_(handle, "PixelFormat") ) );
     //There is a list for the remaining attributes
     for( propertyMap_t::iterator i = propertyMap.begin(), iend = propertyMap.end(); i != iend; ++i ){
        attributes.insert(std::make_pair(i->right,pg_get_attribute_(handle,i->right))); 
     }

    return attributes;
}

/*
static void pv_set_attribute_( tPvHandle& handle, const std::string& key, const std::string& value )
{
    tPvAttributeInfo info;
    if( PvAttrIsAvailable( handle, key.c_str() ) != ePvErrSuccess ) { COMMA_THROW( comma::exception, "attribute \"" << key << "\" unavailable" ); }
    if( PvAttrInfo( handle, key.c_str(), &info ) != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to get attribute info for \"" << key << "\"" ); }
    tPvErr result;
    switch( info.Datatype )
    {
        case ePvDatatypeString:
        case ePvDatatypeEnum:
            result = PvAttrEnumSet( handle, key.c_str(), value.c_str() );
            break;
        case ePvDatatypeUint32:
            result = PvAttrUint32Set( handle, key.c_str(), boost::lexical_cast< int >( value ) );
            break;
        case ePvDatatypeFloat32:
            result = PvAttrFloat32Set( handle, key.c_str(), boost::lexical_cast< int >( value ) );
            break;
        case ePvDatatypeCommand:
            result = PvCommandRun( handle, key.c_str() );
            break;
        case ePvDatatypeRaw:
        case ePvDatatypeUnknown:
        default:
            COMMA_THROW( comma::exception, "unknown attribute \"" << key << "\"" );
    };
    if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to set attribute \"" << key << "\": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
}*/



static cv::Mat pg_as_cvmat_( const Image& frame )
{
    int type;
    switch( frame.GetPixelFormat() )
    {
        case PIXEL_FORMAT_MONO8:
        case PIXEL_FORMAT_RAW8:
            type = CV_8UC1;
            break;
        case PIXEL_FORMAT_RAW16:
        case PIXEL_FORMAT_MONO16:
            type = CV_16UC1;
            break;
        case PIXEL_FORMAT_RGB:
        case PIXEL_FORMAT_BGR:
            type = CV_8UC3;
            break;
        case PIXEL_FORMAT_RGBU:
        case PIXEL_FORMAT_BGRU:
            type = CV_8UC4;
            break;
         case PIXEL_FORMAT_RGB16:
            type = CV_16UC3;
            break;
        case PIXEL_FORMAT_411YUV8:
        case PIXEL_FORMAT_422YUV8:
        case PIXEL_FORMAT_444YUV8:
            COMMA_THROW( comma::exception, "unsupported format " << frame.GetPixelFormat()  );
        default:
            COMMA_THROW( comma::exception, "unknown format " << frame.GetPixelFormat()  );
    };

    return cv::Mat( frame.GetRows(), frame.GetCols(), type, frame.GetData() );
}

class flycapture::impl
{
    //Note, for Point Grey, the serial number is used as ID
    public:
        impl( unsigned int id, const attributes_type& attributes ) :
            started_( false ),
            timeOut_( 1000 )
        {
            initialize_();
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            unsigned int size = 0;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                const std::vector< CameraInfo >& list = list_cameras();
                size = list.size();

                for( unsigned int i = 0; i < list.size(); ++i ) // look for a point grey camera that matches the serial number
                {
                    if(list[i].interfaceType == INTERFACE_GIGE
                        && ( id == 0 || id == list[i].serialNumber ) )
                    {
                        id_ = list[i].serialNumber;
                        break;
                    }
                }
                if( id_ ) { break; }
             }
             if( !id_ ) { COMMA_THROW( comma::exception, "timeout; camera not found" ); }

             if( id == 0 && size > 1 )
             {
                const std::vector< CameraInfo >& list = list_cameras();
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
            BusManager busMgr;
            busMgr.GetCameraFromSerialNumber(*id_, &guid);
    
            Error result = handle_.Connect(&guid);
            for( ; ( result != PGRERROR_OK ) && ( now < end ); now = boost::posix_time::microsec_clock::universal_time() )
            {
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
                result = handle_.Connect(&guid);
            }

            if (result != PGRERROR_OK){close(); COMMA_THROW( comma::exception, "failed to open point grey camera: " << result.GetDescription() );}
//              for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
//              {
//                 pv_set_attribute_( handle_, i->first, i->second );
//             }
    
        }

        ~impl() { close(); }

        void close()
        {
            id_.reset();
            if( !handle_.IsConnected() ) { return; }
            handle_.StopCapture();
            handle_.Disconnect();
            //std::cerr << "the camera has been closed" << std::endl;
        }

        std::pair< boost::posix_time::ptime, cv::Mat > read()
        {
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            bool success = false;
            unsigned int retries = 0;
            while( !success && retries < maxRetries )
            {
                Error result;
                if( !started_ )
                {
                    result = handle_.StartCapture();
                }
                 // error is not checked as sometimes the camera
                 // will start correctly but return an error
                started_ = true;
                Image rawImage;
                result = handle_.RetrieveBuffer(&rawImage);
                frame_.DeepCopy(&rawImage);
                rawImage.ReleaseBuffer();
                total_bytes_per_frame_ = frame_.GetDataSize();
                pair.first = boost::posix_time::microsec_clock::universal_time();
  
                if(( result == PGRERROR_OK))
                {
                    pair.second =  pg_as_cvmat_( frame_ );
                    success = true;
                } 
                else if( //These are errors that result in a retry
                    (result == PGRERROR_ISOCH_START_FAILED )
                    | (result == PGRERROR_TIMEOUT )
                    | (result==PGRERROR_ISOCH_ALREADY_STARTED)
                    | (result==PGRERROR_UNDEFINED) 
                    | (result==PGRERROR_IIDC_FAILED) /*error 22*/
                    | (result==PGRERROR_IMAGE_CONSISTENCY_ERROR))
                {
                    std::cerr << "Error: " << result.GetDescription() << " Retrying..." << std::endl;
                    handle_.StopCapture();
                    started_ = false;
                }
                 else
                {
                    COMMA_THROW( comma::exception, "got frame with invalid status on camera " << *id_ << ": " << result.GetType() << ": " << result.GetDescription() );
                }
                retries++;
             }
             if( success ) { return pair; }
             COMMA_THROW( comma::exception, "got lots of missing frames or timeouts" << std::endl << std::endl << "it is likely that MTU size on your machine is less than packet size" << std::endl << "check PacketSize attribute (flycapture-cat --list-attributes)" << std::endl << "set packet size (e.g. flycapture-cat --set=PacketSize=1500)" << std::endl << "or increase MTU size on your machine" );
        }
        
         const GigECamera& handle() const { return handle_; }
 
         GigECamera& handle() { return handle_; }

        unsigned int id() const { return *id_; }

        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }

        static std::vector< CameraInfo > list_cameras()
        {
            initialize_();
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            std::vector< CameraInfo > list;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                BusManager busMgr;
                unsigned int numCameras;
                Error error;
                error = busMgr.GetNumOfCameras(&numCameras);
                if (error != PGRERROR_OK){
                    COMMA_THROW( comma::exception, "Cannot find point grey cameras");
                }

                CameraInfo camInfo[numCameras];
                error = BusManager::DiscoverGigECameras( camInfo, &numCameras );
                if (error != PGRERROR_OK){
                    COMMA_THROW( comma::exception, "Cannot discover point grey cameras");
                }
                
                //If array is not empty, convert to list and exit
                if( numCameras > 0 ) {
                    std::copy(&camInfo[0],&camInfo[numCameras],std::back_inserter(list));
                    break;
                }
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
            }
            return list;
        }
        
    private:
        friend class flycapture::callback::impl;
        GigECamera handle_;
        Image frame_;
        std::vector< char > buffer_;
        boost::optional< unsigned int > id_;
        PGRGuid guid;
        unsigned long total_bytes_per_frame_;
        bool started_;
        unsigned int timeOut_; // milliseconds
        static void initialize_() // quick and dirty
        {
            pg_create_PixelFormat_map_();
            pg_create_property_map_();
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
// 	       handle.StopCapture();
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

flycapture::flycapture( unsigned int id, const flycapture::attributes_type& attributes ) : pimpl_( new impl( id, attributes ) ) {}

flycapture::~flycapture() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > flycapture::read() { return pimpl_->read(); }

void flycapture::close() { pimpl_->close(); }

std::vector< CameraInfo > flycapture::list_cameras() { return flycapture::impl::list_cameras(); }

unsigned int flycapture::id() const { return pimpl_->id(); }

unsigned long flycapture::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

flycapture::attributes_type flycapture::attributes() const { return pg_attributes_( pimpl_->handle() ); }

flycapture::callback::callback( flycapture& flycapture, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame )
    : pimpl_( new callback::impl( flycapture, on_frame ) )
{
}

flycapture::callback::~callback() { delete pimpl_; }

bool flycapture::callback::good() const { return pimpl_->good; }

} }// namespace snark{ namespace camera{ 
