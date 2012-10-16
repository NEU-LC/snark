// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <comma/base/exception.h>
#include "./gige.h"

static void PVDECL pv_callback_( tPvFrame *frame );

namespace snark{ namespace camera{

static const unsigned int timeOutFactor = 3;
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
    };
    return cv::Mat( frame.Height, frame.Width, type, frame.ImageBuffer );
}

class gige::impl
{
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
            buffer_.resize( total_bytes_per_frame_ );
            frame_.ImageBuffer = &buffer_[0];
            frame_.ImageBufferSize = total_bytes_per_frame_;
            float frameRate;
            PvAttrFloat32Get( handle_, "FrameRate", &frameRate);
            timeOut_ = 1000 / frameRate;
            timeOut_ *= timeOutFactor;
        }

        ~impl() { close(); }

        void close()
        {
            id_.reset();
            if( !handle_ ) { return; }
            PvCameraClose( handle_ );
            PvUnInitialize();
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
                    COMMA_THROW( comma::exception, "got frame with invalid status on camera " << *id_ << ": " << pv_error_to_string_( result ) << "(" << result << ")" );
                    close();                    
                }
                retries++;
            }
            if( success ) { return pair; }
            COMMA_THROW( comma::exception, "got lots of missing frames or timeouts" << std::endl << std::endl << "it is likely that MTU size on your machine is less than packet size" << std::endl << "check packetSize attribute (gige-cat --list-attributes)" << std::endl << "set packet size (e.g. gige-cat --set=packetSize=1500)" << std::endl << "or increase MTU size on your machine" );
        }
        
        const tPvHandle& handle() const { return handle_; }
        
        tPvHandle& handle() { return handle_; }

        unsigned int id() const { return *id_; }

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
        bool started_;
        unsigned int timeOut_; // milliseconds
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

gige::gige( unsigned int id, const gige::attributes_type& attributes ) : pimpl_( new impl( id, attributes ) ) {}

gige::~gige() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > gige::read() { return pimpl_->read(); }

void gige::close() { pimpl_->close(); }

std::vector< tPvCameraInfo > gige::list_cameras() { return gige::impl::list_cameras(); }

unsigned int gige::id() const { return pimpl_->id(); }

unsigned long gige::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

gige::attributes_type gige::attributes() const { return pv_attributes_( pimpl_->handle() ); }

gige::callback::callback( gige& gige, boost::function< void ( std::pair< boost::posix_time::ptime, cv::Mat > ) > on_frame )
    : pimpl_( new callback::impl( gige, on_frame ) )
{
}

gige::callback::~callback() { delete pimpl_; }

bool gige::callback::good() const { return pimpl_->good; }

} } // namespace snark{ namespace camera{
