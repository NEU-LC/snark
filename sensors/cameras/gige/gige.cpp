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
#include <comma/base/types.h>
#include "gige.h"

static void PVDECL pv_callback_( tPvFrame *frame );

namespace snark{ namespace camera{

static const unsigned int timeOutFactor = 3;
static const unsigned int timeOutMaxAddition = 500;
static const unsigned int maxRetries = 15;

static const char *pv_error_to_string_( tPvErr error )
{
    switch( error )
    {
        case ePvErrSuccess: return "ePvErrSuccess";
        case ePvErrCameraFault: return "ePvErrCameraFault";
        case ePvErrInternalFault: return "ePvErrInternalFault";
        case ePvErrBadHandle: return "ePvErrBadHandle";
        case ePvErrBadParameter: return "ePvErrBadParameter";
        case ePvErrBadSequence: return "ePvErrBadSequence";
        case ePvErrNotFound: return "ePvErrNotFound";
        case ePvErrAccessDenied: return "ePvErrAccessDenied";
        case ePvErrUnplugged: return "ePvErrUnplugged";
        case ePvErrInvalidSetup: return "ePvErrInvalidSetup";
        case ePvErrResources: return "ePvErrResources";
        case ePvErrBandwidth: return "ePvErrBandwidth";
        case ePvErrQueueFull: return "ePvErrQueueFull";
        case ePvErrBufferTooSmall: return "ePvErrBufferTooSmall";
        case ePvErrCancelled: return "ePvErrCancelled";
        case ePvErrDataLost: return "ePvErrDataLost";
        case ePvErrDataMissing: return "ePvErrDataMissing";
        case ePvErrTimeout: return "ePvErrTimeout";
        case ePvErrOutOfRange: return "ePvErrOutOfRange";
        case ePvErrWrongType: return "ePvErrWrongType";
        case ePvErrForbidden: return "ePvErrForbidden";
        case ePvErrUnavailable: return "ePvErrUnavailable";
        case ePvErrFirewall: return "ePvErrFirewall";
        case __ePvErr_force_32: return "__ePvErr_force_32";
        default : return "unknown error";
    };
}

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
}

static std::string pv_get_attribute_( tPvHandle& handle, const std::string& key )
{
    if( PvAttrIsAvailable( handle, key.c_str() ) != ePvErrSuccess ) { COMMA_THROW( comma::exception, "attribute \"" << key << "\" unavailable" ); }
    tPvAttributeInfo info;
    if( PvAttrInfo( handle, key.c_str(), &info ) != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to get attribute info for \"" << key << "\"" ); }
    switch( info.Datatype )
    {
        case ePvDatatypeString:
        case ePvDatatypeEnum:
        {
            std::vector< char > buf( 256 ); // arbitrary
            unsigned long n;
            PvAttrEnumGet( handle, key.c_str(), &buf[0], buf.size(), &n );
            return std::string( &buf[0] );
        }
        case ePvDatatypeUint32:
        {
            tPvUint32 n;
            PvAttrUint32Get( handle, key.c_str(), &n );
            return boost::lexical_cast< std::string >( n );
        }
        case ePvDatatypeFloat32:
        {
            tPvFloat32 f;
            PvAttrFloat32Get( handle, key.c_str(), &f );
            return boost::lexical_cast< std::string >( f );
        }
        case ePvDatatypeRaw:
        case ePvDatatypeUnknown:
        default:
            return ""; //COMMA_THROW( comma::exception, "unknown attribute \"" << key << "\"" );
    };
}

gige::attributes_type pv_attributes_( tPvHandle& handle )
{
    tPvAttrListPtr list;
    unsigned long size;
    tPvErr result = PvAttrList( handle, &list, &size );
    if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to get attribute list: " << pv_error_to_string_( result ) << " (" << result << ")" ); }
    gige::attributes_type attributes;
    for( unsigned int i = 0; i < size; ++i ) { attributes.insert( std::make_pair( list[i], pv_get_attribute_( handle, list[i] ) ) ); }
    return attributes;
}

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
        case ePvFmtBayer12Packed:
        {
            if(frame.ImageBufferSize != frame.Height*frame.Width*1.5) 
            { 
                COMMA_THROW( comma::exception, "12bit packed size mismatch; frame.{ ImageBufferSize" << frame.ImageBufferSize
                    <<" frame.Height "<<frame.Height <<" frame.Width "<<frame.Width<<"}");
                
            }
            return cv::Mat( frame.Height, frame.Width*1.5, CV_8UC1, frame.ImageBuffer );
        }
    };
    return cv::Mat( frame.Height, frame.Width, type, frame.ImageBuffer );
}

class gige::impl
{
    public:
        impl( unsigned int id, bool use_camera_timestamp, const attributes_type& attributes )
            : started_( false )
            , timeOut_( 1000 )
            , use_camera_timestamp_( use_camera_timestamp )
        {
            initialize_();
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            unsigned int size = 0;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                const std::vector< tPvCameraInfo >& list = list_cameras();
                size = list.size();
                for( unsigned int i = 0; i < list.size(); ++i ) // look for a gige camera which permits Master Access
                {
                    if(    list[i].InterfaceType == ePvInterfaceEthernet
                        && ( attributes.empty() || list[i].PermittedAccess & ePvAccessMaster )
                        && ( id == 0 || id == list[i].UniqueId ) )
                    {
                        id_ = list[i].UniqueId;
                        break;
                    }
                }
                if( id_ ) { break; }
            }
            if( !id_ ) { COMMA_THROW( comma::exception, "timeout; camera not found" ); }
            if( id == 0 && size > 1 )
            {
                const std::vector< tPvCameraInfo >& list = list_cameras();
                std::stringstream stream;
                for( std::size_t i = 0; i < list.size(); ++i ) // todo: serialize properly with name-value
                {
                    stream << "id=" << list[i].UniqueId << "," << "name=\"" << list[i].DisplayName << "\"" << "," << "serial=\"" << list[i].SerialString << "\"" << std::endl;
                }
                COMMA_THROW( comma::exception, "no id provided and multiple cameras found: would pick up a random one\n\tavailable cameras:\n" << stream.str() );
            }
            now = boost::posix_time::microsec_clock::universal_time();
            end = now + timeout;
            tPvErr result = PvCameraOpen( *id_, ePvAccessMaster, &handle_ );
            for( ; ( result != ePvErrSuccess ) && ( now < end ); now = boost::posix_time::microsec_clock::universal_time() )
            {
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
                result = PvCameraOpen( *id_, ePvAccessMaster, &handle_ );
            }
            if( result != ePvErrSuccess ) { close(); COMMA_THROW( comma::exception, "failed to open gige camera: " << pv_error_to_string_( result ) << " (" << result << ")" ); }
            for( attributes_type::const_iterator i = attributes.begin(); i != attributes.end(); ++i )
            {
                pv_set_attribute_( handle_, i->first, i->second );
            }
            PvAttrEnumSet( handle_, "AcquisitionMode", "Continuous" );
            PvAttrEnumSet( handle_, "FrameStartTriggerMode", "FixedRate" );
            ::memset( &frame_, 0, sizeof( tPvFrame ) ); // voodoo
            result = PvAttrUint32Get( handle_, "TotalBytesPerFrame", &total_bytes_per_frame_ );
            if( result != ePvErrSuccess ) { close(); COMMA_THROW( comma::exception, "failed to get TotalBytesPerFrame from gige camera " << *id_ << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
            if( use_camera_timestamp_ ) // quick and dirty, is it even correct?
            {
                tPvUint32 v;
                if( PvAttrUint32Get( handle_, "TimeStampFrequency", &v ) != ePvErrSuccess ) { close(); COMMA_THROW( comma::exception, "failed to get TimeStampFrequency from gige camera " << *id_ << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
                timestamp_frequency_ = v;
                double ticks;
                start_time_ = boost::posix_time::microsec_clock::universal_time();
                if( PvAttrUint32Get( handle_, "TimeStampValueHi", &v ) != ePvErrSuccess) { close(); COMMA_THROW( comma::exception, "failed to get TimeStampValueHi from gige camera " << *id_ << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
                ticks = comma::uint64( v ) << 32;
                if( PvAttrUint32Get( handle_, "TimeStampValueLo", &v ) != ePvErrSuccess) { close(); COMMA_THROW( comma::exception, "failed to get TimeStampValueLo from gige camera " << *id_ << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
                ticks += v;
                double elapsed = ticks / timestamp_frequency_;
                start_time_ -= ( boost::posix_time::seconds( comma::uint32( elapsed ) ) + boost::posix_time::microseconds( static_cast< long >( ( elapsed - comma::uint32( elapsed ) ) * 1000000 ) ) );
            }
            buffer_.resize( total_bytes_per_frame_ );
            frame_.ImageBuffer = &buffer_[0];
            frame_.ImageBufferSize = total_bytes_per_frame_;
            // calculate sensible default timeout from camera settings
            tPvFloat32 frameRate;
            PvAttrFloat32Get( handle_, "FrameRate", &frameRate );
            tPvUint32 exposureValue;
            PvAttrUint32Get( handle_, "ExposureValue", &exposureValue );
            float effectiveMinFramerate = 1.0e6 / exposureValue;
            std::string exposureMode = pv_get_attribute_( handle_, "ExposureMode" );
            if ( exposureMode == "Auto" || exposureMode == "AutoOnce" )
            {
                tPvUint32 exposureAutoMax;
                PvAttrUint32Get( handle_, "ExposureAutoMax", &exposureAutoMax );
                effectiveMinFramerate = 1.0e6 / exposureAutoMax;
                if ( effectiveMinFramerate < frameRate ) { std::cerr << "Warning: ExposureAutoMax=" << exposureAutoMax << " may limit effecitve frame rate to " << effectiveMinFramerate << " in low light conditions (less than configured FrameRate=" << frameRate << ")." << std::endl; }
                // also ensure initial exposure setting doesn't immediately cause timeouts unnecessarily
                if ( exposureValue > exposureAutoMax ) { PvAttrUint32Set( handle_, "ExposureValue", exposureAutoMax ); }
            }
            else if ( exposureMode == "Manual" || exposureMode == "PieceWiseLinearHDR" )
            {
                if ( effectiveMinFramerate < frameRate ) { std::cerr << "Warning: ExposureValue=" << exposureValue << " limits effective frame rate to " << effectiveMinFramerate << " (less than configured FrameRate=" << frameRate << ")." << std::endl; }
            }
            else
            {
                // else exposureMode is External or something else as yet undocumented
                // this probably requires an explicit timeout setting to be provided
            }
            // timeout up to 3x max time between frames, capped at frame period + 500ms
            timeOut_ = 1000.0 / std::min( frameRate, effectiveMinFramerate );
            timeOut_ = std::min( timeOut_ * timeOutFactor, timeOut_ + timeOutMaxAddition );
            // std::cerr << "Frame timeout set to " << timeOut_ << " ms" << std::endl;
        }

        ~impl() { close(); }

        void close()
        {
            id_.reset();
            if( !handle_ ) { return; }
            PvCameraClose( handle_ );
            PvUnInitialize();
            //std::cerr << "the camera has been closed" << std::endl;
        }
        
        boost::posix_time::ptime timestamp_( comma::uint32 hi, comma::uint32 lo )
        {
            double s = double( ( comma::uint64( hi ) << 32 ) + lo ) / timestamp_frequency_;
            comma::uint32 seconds = s;
            comma::uint32 microseconds = ( double( s ) - seconds ) * 1000000;
            return start_time_ + boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( microseconds );
        }

        std::pair< boost::posix_time::ptime, cv::Mat > read()
        {
            std::pair< boost::posix_time::ptime, cv::Mat > pair;
            bool success = false;
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
                            pair.first = use_camera_timestamp_
                                       ? timestamp_( frame_.TimestampHi, frame_.TimestampLo )
                                       : boost::posix_time::microsec_clock::universal_time();
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
                    COMMA_THROW( comma::exception, "got frame with invalid status on camera " << *id_ << ": " << pv_error_to_string_( result ) << "(" << result << ")" );
                }
                retries++;
            }
            if( success ) { return pair; }
            COMMA_THROW( comma::exception, "got lots of missing frames or timeouts" << std::endl << std::endl << "it is likely that MTU size on your machine is less than packet size" << std::endl << "check PacketSize attribute (gige-cat --list-attributes)" << std::endl << "set packet size (e.g. gige-cat --set=PacketSize=1500)" << std::endl << "or increase MTU size on your machine" );
        }

        const tPvHandle& handle() const { return handle_; }

        tPvHandle& handle() { return handle_; }

        unsigned int id() const { return *id_; }

        void set_frame_timeout( unsigned int timeout ) { timeOut_ = timeout; }

        unsigned int frame_timeout() const { return timeOut_; }

        unsigned long total_bytes_per_frame() const { return total_bytes_per_frame_; }

        static std::vector< tPvCameraInfo > list_cameras()
        {
            initialize_();
            static const boost::posix_time::time_duration timeout = boost::posix_time::seconds( 5 ); // quick and dirty; make configurable?
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::ptime end = now + timeout;
            std::vector< tPvCameraInfo > list;
            for( ; now < end; now = boost::posix_time::microsec_clock::universal_time() )
            {
                std::size_t count = PvCameraCount();
                list.resize( count );
                count = PvCameraList( &list[0], count, NULL );
                list.resize( count );
                if( !list.empty() ) { break; }
                boost::thread::sleep( now + boost::posix_time::milliseconds( 10 ) );
            }
            return list;
        }

    private:
        friend class gige::callback::impl;
        tPvHandle handle_;
        tPvFrame frame_;
        std::vector< char > buffer_;
        boost::optional< unsigned int > id_;
        unsigned long total_bytes_per_frame_;
        comma::uint32 timestamp_frequency_;
        boost::posix_time::ptime start_time_;
        bool started_;
        unsigned int timeOut_; // milliseconds
        bool use_camera_timestamp_;
        static void initialize_() // quick and dirty
        {
            static tPvErr result = PvInitialize(); // should it be a singleton?
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to initialize gige camera: " << pv_error_to_string_( result ) << " (" << result << ")" ); }
        }
};

class gige::callback::impl
{
    public:
        typedef boost::function< void ( const std::pair< boost::posix_time::ptime, cv::Mat >& ) > OnFrame;

        impl( gige& gige, OnFrame on_frame )
            : on_frame( on_frame )
            , handle( gige.pimpl_->handle() )
            , frame( gige.pimpl_->frame_ )
            , good( true )
            , is_shutdown( false )
        {
            tPvErr result;
            PvCaptureQueueClear( handle );
            frame.Context[0] = this;
            result = PvCaptureStart( handle );
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to start capturing on camera " << gige.pimpl_->id() << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
            result = PvCaptureQueueFrame( handle, &frame, pv_callback_ );
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to set capture queue frame on camera " << gige.pimpl_->id() << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
            result = PvCommandRun( handle, "AcquisitionStart" );
            if( result != ePvErrSuccess ) { COMMA_THROW( comma::exception, "failed to start acquisition on camera " << gige.pimpl_->id() << ": " << pv_error_to_string_( result ) << " (" << result << ")" ); }
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

} } // namespace snark{ namespace camera{

static void PVDECL pv_callback_( tPvFrame *frame )
{
    snark::camera::gige::callback::impl* c = reinterpret_cast< snark::camera::gige::callback::impl* >( frame->Context[0] );
    if( c->is_shutdown ) { return; }
    std::pair< boost::posix_time::ptime, cv::Mat > m( boost::posix_time::microsec_clock::universal_time(), cv::Mat() );
    if( frame ) { m.second = snark::camera::pv_as_cvmat_( *frame ); }
    c->on_frame( m );
    tPvErr result = PvCaptureQueueFrame( c->handle, &c->frame, pv_callback_ );
    if( result != ePvErrSuccess ) { c->good = false; }
}

namespace snark{ namespace camera{

gige::gige( unsigned int id, const gige::attributes_type& attributes ) : pimpl_( new impl( id, false, attributes ) ) {}

gige::gige( unsigned int id, bool use_camera_timestamp, const attributes_type& attributes ) : pimpl_( new impl( id, false, attributes ) ) {}

gige::~gige() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > gige::read() { return pimpl_->read(); }

void gige::close() { pimpl_->close(); }

std::vector< tPvCameraInfo > gige::list_cameras() { return gige::impl::list_cameras(); }

unsigned int gige::id() const { return pimpl_->id(); }

void gige::set_frame_timeout( unsigned int timeout ) { pimpl_->set_frame_timeout(timeout); }

unsigned int gige::frame_timeout() const { return pimpl_->frame_timeout(); }

unsigned long gige::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

gige::attributes_type gige::attributes() const { return pv_attributes_( pimpl_->handle() ); }

gige::callback::callback( gige& gige, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame )
    : pimpl_( new callback::impl( gige, on_frame ) )
{
}

gige::callback::~callback() { delete pimpl_; }

bool gige::callback::good() const { return pimpl_->good; }

} } // namespace snark{ namespace camera{
