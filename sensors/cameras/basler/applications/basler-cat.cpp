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

/// @author vsevolod vlaskine

#include <stdlib.h>
#include <boost/thread/thread_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include <comma/visiting/traits.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../../../../imaging/cv_mat/bursty_pipeline.h"

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <pylon/usb/BaslerUsbCamera.h>

static double default_timeout = 3.0;
static const char* possible_header_fields = "t,rows,cols,type,size,counters";
static const char* default_header_fields = "t,rows,cols,type";

static void usage( bool verbose = false )
{
    std::cerr << "\nacquire images from a basler camera";
    std::cerr << "\noutput to stdout as serialized cv::Mat";
    std::cerr << "\n";
    std::cerr << "\nusage: basler-cat [<options>] [<filters>]";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h                 display help message";
    std::cerr << "\n    --address=<address>       camera ip address; default: first available";
    std::cerr << "\n    --discard                 discard frames, if cannot keep up;";
    std::cerr << "\n                              same as --buffer=1";
    std::cerr << "\n    --buffer=<buffers>        maximum buffer size before discarding frames";
    std::cerr << "\n                              default: unlimited";
    std::cerr << "\n    --list-cameras            output camera list and exit";
    std::cerr << "\n    --fields,-f=<fields>      header fields, possible values:";
    std::cerr << "\n                              possible values: " << possible_header_fields;
    std::cerr << "\n                              default: " << default_header_fields;
    std::cerr << "\n    --camera-type=<type>      camera type: gige or usb; default: gige";
    std::cerr << "\n    --image-type=<type>       image type; default: 3ub; --verbose for more";
    std::cerr << "\n    --offset-x=<pixels>       offset in pixels in the line";
    std::cerr << "\n    --offset-y=<pixels>       offset in lines in the frame";
    std::cerr << "\n    --width=<pixels>          line width in pixels; default: max";
    std::cerr << "\n    --height=<pixels>         number of lines in frame (in chunk mode always 1)";
    std::cerr << "\n                              default: max";
    std::cerr << "\n    --frame-trigger=<trigger> 'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-trigger=<trigger>  'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-rate=<num>         line acquisition rate";
    std::cerr << "\n    --encoder-ticks=<num>     number of encoder ticks until the count resets";
    std::cerr << "\n                              (reused for line number in frame in chunk mode)";
    std::cerr << "\n    --header-only             output header only";
    std::cerr << "\n    --no-header               output image data only";
    std::cerr << "\n    --packet-size=<bytes>     mtu size on camera side, should not be larger ";
    std::cerr << "\n                              than your lan and network interface";
    std::cerr << "\n    --exposure=<num>          exposure";
    std::cerr << "\n    --gain=<num>              gain";
    std::cerr << "\n    --timeout=<seconds>       frame acquisition timeout; default " << default_timeout << "s";
    std::cerr << "\n    --test-colour             output colour test image";
    std::cerr << "\n    --verbose,-v              be more verbose";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\n";
        std::cerr << snark::cv_mat::filters::usage();
        std::cerr << snark::cv_mat::serialization::options::type_usage();
    }
    std::cerr << "\nnote: there is a glitch or a subtle feature in basler line camera:";
    std::cerr << "\n    - power-cycle camera";
    std::cerr << "\n    - view colour images: it works";
    std::cerr << "\n    - view grey-scale images: it works";
    std::cerr << "\n    - view colour images: it still displays grey-scale";
    std::cerr << "\n";
    std::cerr << "\n    even in their native viewer you need to set colour image repeatedly and";
    std::cerr << "\n    with pure luck it works, but we have not managed to do it in software;";
    std::cerr << "\n    the remedy: power-cycle the camera";
    std::cerr << "\n" << std::endl;
}

struct ChunkData
{
    boost::posix_time::ptime timestamp;
    comma::uint32 frames;
    comma::uint64 ticks; // timestamp; 1 tick = 8ns
    comma::uint32 line_trigger_ignored;
    comma::uint32 frame_trigger_ignored;
    comma::uint32 line_trigger_end_to_end;
    comma::uint32 frame_trigger;
    comma::uint32 frames_per_trigger;
};

struct Counters
{
    boost::posix_time::ptime adjusted_timestamp;
    comma::uint64 ticks; // number of 8ns ticks
    comma::uint64 line_count; // overall line count
    comma::uint32 line; // line number in one motor revolution
    
    Counters() : line( 0 ), ticks( 0 ), line_count( 0 ) {}
};

struct Header // quick and dirty
{
    snark::cv_mat::serialization::header header;
    Counters counters;
    
    Header() {}
    Header( const snark::cv_mat::serialization::header& header ) : header( header ) {}
};

static snark::cv_mat::serialization::options cv_mat_options;
static comma::csv::options csv;
static bool verbose;
static bool is_shutdown = false;
static bool done = false;
static unsigned int timeout;
static Pylon::IChunkParser* parser = NULL;
static unsigned int encoder_ticks;

namespace comma { namespace visiting {

template <> struct traits< Counters >
{
    template < typename K, typename V >
    static void visit( const K&, Counters& t, V& v )
    {
        v.apply( "adjusted-t", t.adjusted_timestamp );
        v.apply( "ticks", t.ticks );
        v.apply( "line-count", t.line_count );
        v.apply( "line", t.line );
    }
    
    template < typename K, typename V >
    static void visit( const K&, const Counters& t, V& v )
    {
        v.apply( "adjusted-t", t.adjusted_timestamp );
        v.apply( "ticks", t.ticks );
        v.apply( "line-count", t.line_count );
        v.apply( "line", t.line );
    }
};    
    
template <> struct traits< Header >
{
    template < typename K, typename V >
    static void visit( const K&, Header& t, V& v )
    {
        v.apply( "header", t.header );
        v.apply( "counters", t.counters );
    }
    
    template < typename K, typename V >
    static void visit( const K&, const Header& t, V& v )
    {
        v.apply( "header", t.header );
        v.apply( "counters", t.counters );
    }
};
    
} } // namespace comma { namespace visiting {

typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;
typedef std::pair< ChunkData, cv::Mat > ChunkPair;    

namespace gige {

static void set_( boost::posix_time::ptime& timestamp
                , boost::posix_time::ptime& t
                , const Pylon::GrabResult&
                , Pylon::CBaslerGigECamera& )
{
    timestamp = t;
}

static void set_( ChunkData& d
                , boost::posix_time::ptime& t
                , const Pylon::GrabResult& result
                , Pylon::CBaslerGigECamera& camera )
{
    parser->AttachBuffer( ( unsigned char* ) result.Buffer(), result.GetPayloadSize() );
    d.timestamp = t;
    d.frames = camera.ChunkFramecounter.GetValue();
    d.ticks = camera.ChunkTimestamp.GetValue();
    d.line_trigger_ignored = camera.ChunkLineTriggerIgnoredCounter.GetValue();
    d.frame_trigger_ignored = camera.ChunkFrameTriggerIgnoredCounter.GetValue();
    d.line_trigger_end_to_end = camera.ChunkLineTriggerEndToEndCounter.GetValue();
    d.frame_trigger = camera.ChunkFrameTriggerCounter.GetValue();
    d.frames_per_trigger = camera.ChunkFramesPerTriggerCounter.GetValue();
    parser->DetachBuffer();
}

template < typename P >
static P capture_( Pylon::CBaslerGigECamera& camera, Pylon::CBaslerGigECamera::StreamGrabber_t& grabber )
{
    /// @todo if color spatial correction implemented, mind the following:
    ///
    /// Runner_Users_manual.pdf, 8.2.2.3:
    ///
    /// If you are using a color camera, you have spatial correction enabled
    /// and you have the frame start trigger mode set to off, you must discard
    /// the first n x 2 lines from the first frame transmitted by the camera
    /// after an acquisition start command is issued (where n is the absolute
    /// value of the current spatial correction parameter setting).
    ///
    /// If you have spatial correction enabled and you have the frame start
    /// trigger mode set to on, you must discard the first n x 2 lines from
    /// each frame transmitted by the camera.
    
    static const unsigned int retries = 10; // quick and dirty: arbitrary
    for( unsigned int i = 0; !done && i < retries; ++i )
    {
        Pylon::GrabResult result;
        //camera.AcquisitionStart.Execute(); // acquire single image (since acquisition mode set so)
        if( !grabber.GetWaitObject().Wait( timeout ) ) // quick and dirty: arbitrary timeout
        {
            std::cerr << "basler-cat: timeout" << std::endl;
            grabber.CancelGrab();
            while( grabber.RetrieveResult( result ) ); // get all buffers back
            if( is_shutdown ) { done = true; }
            return P();
        }
        boost::posix_time::ptime t = boost::get_system_time();
        grabber.RetrieveResult( result );
        if( !result.Succeeded() )
        { 
            std::cerr << "basler-cat: acquisition failed: "
                      << result.GetErrorDescription()
                      << " (0x" << std::hex << result.GetErrorCode() << std::dec << ")" << std::endl;
            std::cerr << "            status: " << ( result.Status() == Pylon::Idle ? "idle" :
                                                     result.Status() == Pylon::Queued ? "queued" :
                                                     result.Status() == Pylon::Grabbed ? "grabbed" :
                                                     result.Status() == Pylon::Canceled ? "canceled" :
                                                     result.Status() == Pylon::Failed ? "failed" : "unknown" ) << std::endl;
            std::cerr << "            run basler-cat --verbose and check your --packet-size settings" << std::endl;
            continue;
        }
        P pair;
        static const snark::cv_mat::serialization::header header = cv_mat_options.get_header();
        pair.second = cv::Mat( result.GetSizeY(), result.GetSizeX(), header.type );
        ::memcpy( pair.second.data, reinterpret_cast< const char* >( result.Buffer() )
                , pair.second.dataend - pair.second.datastart );
        switch( header.type )
        {
            case CV_8UC1:
                break;
            case CV_8UC3: // quick and dirty for now: rgb are not contiguous in basler camera frame
                cv::cvtColor( pair.second, pair.second, CV_RGB2BGR );
                break;
            default: // quick and dirty for now
                std::cerr << "basler-cat: cv::mat type " << cv_mat_options.type << " not supported" << std::endl;
        }
        set_( pair.first, t, result, camera );
        grabber.QueueBuffer( result.Handle(), NULL ); // requeue buffer
        if( is_shutdown ) { done = true; }
        return pair;
    }
    if( is_shutdown ) { done = true; }
    return P();
}

static void write_( ChunkPair p )
{
    if( p.second.size().width == 0 || std::cout.bad() || !std::cout.good() || is_shutdown ) { return; }
    static comma::csv::binary_output_stream< Header > ostream( std::cout, csv );
    static Header header( cv_mat_options.get_header() );
    header.header.timestamp = p.first.timestamp;
    header.counters.ticks = p.first.ticks;
    static ChunkData first_chunk_data;
    if( first_chunk_data.timestamp.is_not_a_date_time() )
    {
        first_chunk_data = p.first;
        header.counters.adjusted_timestamp = p.first.timestamp;
    }
    else
    {
        static const double factor = 8.0 / 1000; // 8ns per tick
        header.counters.adjusted_timestamp = first_chunk_data.timestamp + boost::posix_time::microseconds( factor * first_chunk_data.ticks ); // todo: factor in network delay?
    }
    header.counters.line_count += p.first.line_trigger_ignored + 1;
    header.counters.line = header.counters.line_count % encoder_ticks;
    ostream.write( header );
    std::cout.write( ( const char* )( p.second.datastart ), p.second.dataend - p.second.datastart );
    std::cout.flush();
}

static unsigned int set_pixel_format_( Pylon::CBaslerGigECamera& camera, Basler_GigECameraParams::PixelFormatEnums type )
{
    std::string type_string;
    unsigned int channels;
    switch( type )
    {
        case Basler_GigECameraParams::PixelFormat_Mono8: type_string = "Mono8"; channels = 1; break;
        case Basler_GigECameraParams::PixelFormat_RGB8Packed: type_string = "RGB8Packed"; channels = 3; break;
        default: COMMA_THROW( comma::exception, "type " << type << " not implemented" );
    }
    if( camera.PixelFormat.GetValue() == type ) { return channels; }
    GenApi::NodeList_t entries;
    camera.PixelFormat.GetEntries( entries );
    bool supported = false;
    if( verbose ) { std::cerr << "basler-cat: supported pixel format(s): "; }
    for( std::size_t i = 0; i < entries.size(); ++i )
    {
        GenApi::INode* node = entries[i]; // bloody voodoo
        if( !IsAvailable( node->GetAccessMode() ) ) { continue; }
        GenApi::IEnumEntry* e = dynamic_cast< GenApi::IEnumEntry* >( node );
        if( verbose ) { std::cerr << ( i > 0 ? ", " : "" ) << e->GetSymbolic(); }
        supported = supported || type_string != e->GetSymbolic().c_str();
    }
    if( verbose ) { std::cerr << std::endl; }
    if( !supported ) { COMMA_THROW( comma::exception, "pixel format " << type << " is not supported" ); }    
    if( verbose ) { std::cerr << "basler-cat: setting pixel format..." << std::endl; }
    // the voodoo theory is that in the continuous mode the camera settings
    // cannot be changed, while the camera is acquiring a frame. now, you may think:
    // well, just stop acquisition! see below: somehow, stopping acquisition does not
    // work; therefore (even in the native basler viewer) you need to try to set the
    // pixel format between the frames, just trying it until you succeed...
    // and then you try again...
    // but even that does not seem to work... well, power-cycling helps...
    static const unsigned int retries = 100; // quick and dirty: arbitrary
    for( unsigned int i = 0; i < retries; ++i )
    {
        camera.PixelFormat.SetValue( type );
        boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 10 ) );
    }
    if( camera.PixelFormat.GetValue() != type ) { COMMA_THROW( comma::exception, "failed to set pixel format after " << retries << " attempts; try again or power-cycle the camera" ); }
    if( verbose ) { std::cerr << "basler-cat: pixel format set to " << type_string << std::endl; }
    return channels;
}

} // namespace gige

namespace usb {

static void set_( boost::posix_time::ptime& timestamp, boost::posix_time::ptime& t, const Pylon::GrabResult&, Pylon::CBaslerUsbCamera& )
{
    timestamp = t;
}

template < typename P >
static P capture_( Pylon::CBaslerUsbCamera& camera, Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber )
{
    /// @todo if color spatial correction implemented, mind the following:
    ///
    /// Runner_Users_manual.pdf, 8.2.2.3:
    ///
    /// If you are using a color camera, you have spatial correction enabled
    /// and you have the frame start trigger mode set to off, you must discard
    /// the first n x 2 lines from the first frame transmitted by the camera
    /// after an acquisition start command is issued (where n is the absolute
    /// value of the current spatial correction parameter setting).
    ///
    /// If you have spatial correction enabled and you have the frame start
    /// trigger mode set to on, you must discard the first n x 2 lines from
    /// each frame transmitted by the camera.
    
    static const unsigned int retries = 10; // quick and dirty: arbitrary
    for( unsigned int i = 0; !done && i < retries; ++i )
    {
        Pylon::GrabResult result;
        //camera.AcquisitionStart.Execute(); // acquire single image (since acquisition mode set so)
        if( !grabber.GetWaitObject().Wait( timeout ) ) // quick and dirty: arbitrary timeout
        {
            std::cerr << "basler-cat: timeout" << std::endl;
            grabber.CancelGrab();
            while( grabber.RetrieveResult( result ) ); // get all buffers back
            if( is_shutdown ) { done = true; }
            return P();
        }
        boost::posix_time::ptime t = boost::get_system_time();
        grabber.RetrieveResult( result );
        if( !result.Succeeded() )
        { 
            std::cerr << "basler-cat: acquisition failed: "
                      << result.GetErrorDescription()
                      << " (0x" << std::hex << result.GetErrorCode() << std::dec << ")" << std::endl;
            std::cerr << "            status: " << ( result.Status() == Pylon::Idle ? "idle" :
                                                     result.Status() == Pylon::Queued ? "queued" :
                                                     result.Status() == Pylon::Grabbed ? "grabbed" :
                                                     result.Status() == Pylon::Canceled ? "canceled" :
                                                     result.Status() == Pylon::Failed ? "failed" : "unknown" ) << std::endl;
            std::cerr << "            run basler-cat --verbose and check your --packet-size settings" << std::endl;
            continue;
        }
        P pair;
        static const snark::cv_mat::serialization::header header = cv_mat_options.get_header();
        pair.second = cv::Mat( result.GetSizeY(), result.GetSizeX(), header.type );
        ::memcpy( pair.second.data, reinterpret_cast< const char* >( result.Buffer() )
                , pair.second.dataend - pair.second.datastart );
        switch( header.type )
        {
            case CV_8UC1:
                break;
            case CV_8UC3: // quick and dirty for now: rgb are not contiguous in basler camera frame
                cv::cvtColor( pair.second, pair.second, CV_RGB2BGR );
                break;
            default: // quick and dirty for now
                std::cerr << "basler-cat: cv::mat type " << cv_mat_options.type << " not supported" << std::endl;
        }
        set_( pair.first, t, result, camera );
        grabber.QueueBuffer( result.Handle(), NULL ); // requeue buffer
        if( is_shutdown ) { done = true; }
        return pair;
    }
    if( is_shutdown ) { done = true; }
    return P();
}

static void write_( ChunkPair p )
{
    if( p.second.size().width == 0 || std::cout.bad() || !std::cout.good() || is_shutdown ) { return; }
    static comma::csv::binary_output_stream< Header > ostream( std::cout, csv );
    static Header header( cv_mat_options.get_header() );
    header.header.timestamp = p.first.timestamp;
    header.counters.ticks = p.first.ticks;
    static ChunkData first_chunk_data;
    if( first_chunk_data.timestamp.is_not_a_date_time() )
    {
        first_chunk_data = p.first;
        header.counters.adjusted_timestamp = p.first.timestamp;
    }
    else
    {
        static const double factor = 8.0 / 1000; // 8ns per tick
        header.counters.adjusted_timestamp = first_chunk_data.timestamp + boost::posix_time::microseconds( factor * first_chunk_data.ticks ); // todo: factor in network delay?
    }
    header.counters.line_count += p.first.line_trigger_ignored + 1;
    header.counters.line = header.counters.line_count % encoder_ticks;
    ostream.write( header );
    std::cout.write( ( const char* )( p.second.datastart ), p.second.dataend - p.second.datastart );
    std::cout.flush();
}

static unsigned int set_pixel_format_( Pylon::CBaslerUsbCamera& camera, Basler_UsbCameraParams::PixelFormatEnums type )
{
    std::string type_string;
    unsigned int channels;
    switch( type )
    {
        case Basler_UsbCameraParams::PixelFormat_Mono8: type_string = "Mono8"; channels = 1; break;
        default: COMMA_THROW( comma::exception, "type " << type << " not implemented" );
    }
    if( camera.PixelFormat.GetValue() == type ) { return channels; }
    GenApi::NodeList_t entries;
    camera.PixelFormat.GetEntries( entries );
    bool supported = false;
    if( verbose ) { std::cerr << "basler-cat: supported pixel format(s): "; }
    for( std::size_t i = 0; i < entries.size(); ++i )
    {
        GenApi::INode* node = entries[i]; // bloody voodoo
        if( !IsAvailable( node->GetAccessMode() ) ) { continue; }
        GenApi::IEnumEntry* e = dynamic_cast< GenApi::IEnumEntry* >( node );
        if( verbose ) { std::cerr << ( i > 0 ? ", " : "" ) << e->GetSymbolic(); }
        supported = supported || type_string != e->GetSymbolic().c_str();
    }
    if( verbose ) { std::cerr << std::endl; }
    if( !supported ) { COMMA_THROW( comma::exception, "pixel format " << type << " is not supported" ); }    
    if( verbose ) { std::cerr << "basler-cat: setting pixel format..." << std::endl; }
    // the voodoo theory is that in the continuous mode the camera settings
    // cannot be changed, while the camera is acquiring a frame. now, you may think:
    // well, just stop acquisition! see below: somehow, stopping acquisition does not
    // work; therefore (even in the native basler viewer) you need to try to set the
    // pixel format between the frames, just trying it until you succeed...
    // and then you try again...
    // but even that does not seem to work... well, power-cycling helps...
    static const unsigned int retries = 100; // quick and dirty: arbitrary
    for( unsigned int i = 0; i < retries; ++i )
    {
        camera.PixelFormat.SetValue( type );
        boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 10 ) );
    }
    if( camera.PixelFormat.GetValue() != type ) { COMMA_THROW( comma::exception, "failed to set pixel format after " << retries << " attempts; try again or power-cycle the camera" ); }
    if( verbose ) { std::cerr << "basler-cat: pixel format set to " << type_string << std::endl; }
    return channels;
}

} // namespace usb

static unsigned int discard;
static bool chunk_mode = false;
static std::string filters;

namespace gige {

int main( const comma::command_line_options& options )
{
    typedef Pylon::CBaslerGigECamera Camera_t;

    Pylon::PylonAutoInitTerm auto_init_term;
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();
    Pylon::ITransportLayer* transport_layer( Pylon::CTlFactory::GetInstance().CreateTl( Camera_t::DeviceClass() ));
    if( !transport_layer )
    { 
        std::cerr << "basler-cat: failed to create transport layer" << std::endl;
        std::cerr << "            most likely PYLON_ROOT and GENICAM_ROOT_V2_1 environment variables not set" << std::endl;
        std::cerr << "            point them to your pylon installation, e.g:" << std::endl;
        std::cerr << "            export PYLON_ROOT=/opt/pylon" << std::endl;
        std::cerr << "            export GENICAM_ROOT_V2_1=/opt/pylon/genicam" << std::endl;
        return 1;
    }
    if( options.exists( "--list-cameras" ))
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        for( unsigned int i = 0; i < devices.size(); ++i ) { std::cerr << devices[i].GetFullName() << std::endl; }
        return 0;
    }
    timeout = options.value< double >( "--timeout", default_timeout ) * 1000.0;
    Camera_t camera;
    if( options.exists( "--address" ))
    {
        Pylon::CBaslerGigEDeviceInfo info;
        info.SetIpAddress( options.value< std::string >( "--address" ).c_str() );
        camera.Attach( factory.CreateDevice( info ) );
    }
    else
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        if( devices.empty() ) { std::cerr << "basler-cat: no camera found" << std::endl; return 1; }
        std::cerr << "basler-cat: will connect to the first of " << devices.size() << " found device(s):" << std::endl;
        for( unsigned int i = 0; i < devices.size(); ++i ) { std::cerr << "    " << devices[i].GetFullName() << std::endl; }
        camera.Attach( transport_layer->CreateDevice( devices[0] ) );
    }
    if( verbose ) { std::cerr << "basler-cat: initialized camera" << std::endl; }
    if( verbose ) { std::cerr << "basler-cat: opening camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << "..." << std::endl; }
    camera.Open();
    if( verbose ) { std::cerr << "basler-cat: opened camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << std::endl; }
    Camera_t::StreamGrabber_t grabber( camera.GetStreamGrabber( 0 ) );
    grabber.Open();
    unsigned int channels;
    switch( cv_mat_options.get_header().type ) // quick and dirty
    {
        case CV_8UC1:
            channels = set_pixel_format_( camera, Basler_GigECameraParams::PixelFormat_Mono8 );
            break;
        case CV_8UC3:
            channels = set_pixel_format_( camera, Basler_GigECameraParams::PixelFormat_RGB8Packed );
            break;
        default:
            std::cerr << "basler-cat: type \"" << cv_mat_options.type << "\" not implemented or not supported by camera" << std::endl;
            return 1;
    }
    unsigned int max_width = camera.Width.GetMax();
    double offset_x = options.value< double >( "--offset-x", 0 );
    if( offset_x >= max_width ) { std::cerr << "basler-cat: expected --offset-x less than " << max_width << ", got " << offset_x << std::endl; return 1; }
    camera.OffsetX.SetValue( offset_x );
    unsigned int width = options.value< unsigned int >( "--width", max_width );
    width = ( ( unsigned long long )( offset_x ) + width ) < max_width ? width : max_width - offset_x;
    camera.Width.SetValue( width );
    unsigned int max_height = camera.Height.GetMax();
    //if( height < 512 ) { std::cerr << "basler-cat: expected height greater than 512, got " << height << std::endl; return 1; }
        
    // todo: is the colour line 2098 * 3 or ( 2098 / 3 ) * 3 ?
    //offset_y *= channels;
    //height *= channels;

    double offset_y = options.value< double >( "--offset-y", 0 );
    if( offset_y >= max_height ) { std::cerr << "basler-cat: expected --offset-y less than " << max_height << ", got " << offset_y << std::endl; return 1; }
    camera.OffsetY.SetValue( offset_y );
    unsigned int height = options.value< unsigned int >( "--height", max_height );
    height = ( ( unsigned long long )( offset_y ) + height ) < max_height ? height : ( max_height - offset_y );
    camera.Height.SetValue( height );
    if( verbose ) { std::cerr << "basler-cat: set width,height to " << width << "," << height << std::endl; }
    if( options.exists( "--packet-size" )) { camera.GevSCPSPacketSize.SetValue( options.value< unsigned int >( "--packet-size" )); }
    // todo: giving up... the commented code throws, but failure to stop acquisition, if active
    //       seems to lead to the following scenario:
    //       - power-cycle camera
    //       - view colour images: it works
    //       - view grey-scale images: it works
    //       - view colour images: it still displays grey-scale
    //if( verbose ) { std::cerr << "basler-cat: getting acquisition status... (frigging voodoo...)" << std::endl; }
    //GenApi::IEnumEntry* acquisition_status = camera.AcquisitionStatusSelector.GetEntry( Basler_GigECameraParams::AcquisitionStatusSelector_AcquisitionActive );
    //if( acquisition_status && GenApi::IsAvailable( acquisition_status ) && camera.AcquisitionStatus() )
    //{
    //    if( verbose ) { std::cerr << "basler-cat: stopping acquisition..." << std::endl; }
    //    camera.AcquisitionStop.Execute();
    //    if( verbose ) { std::cerr << "basler-cat: acquisition stopped" << std::endl; }
    //}
        
    // todo: a hack for now
    GenApi::IEnumEntry* acquisitionStart = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_AcquisitionStart );
    std::string frame_trigger = options.value< std::string >( "--frame-trigger", "" );
    if( acquisitionStart && GenApi::IsAvailable( acquisitionStart ) )
    {
        camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_AcquisitionStart );
        camera.TriggerMode.SetValue( frame_trigger.empty() ? Basler_GigECameraParams::TriggerMode_Off : Basler_GigECameraParams::TriggerMode_On );
    }
    GenApi::IEnumEntry* frameStart = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_FrameStart );
    if( frameStart && GenApi::IsAvailable( frameStart ) )
    {
        //if( frame_trigger.empty() ) { frame_trigger = line_trigger; }
        if( frame_trigger.empty() )
        {
            camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_FrameStart );
            camera.TriggerMode.SetValue( Basler_GigECameraParams::TriggerMode_Off );
        }
        else
        {
            camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_FrameStart );
            camera.TriggerMode.SetValue( Basler_GigECameraParams::TriggerMode_On );
            Basler_GigECameraParams::TriggerSourceEnums t;
            if( frame_trigger == "line1" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_Line1 ); }
            if( frame_trigger == "line2" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_Line2 ); }
            if( frame_trigger == "line3" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_Line3 ); }
            else if( frame_trigger == "encoder" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut ); }
            else { std::cerr << "basler-cat: frame trigger '" << frame_trigger << "' not implemented or invalid" << std::endl; return 1; }
            camera.TriggerActivation.SetValue( Basler_GigECameraParams::TriggerActivation_RisingEdge );
            camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_LineStart );
            camera.TriggerMode.SetValue( Basler_GigECameraParams::TriggerMode_On );
            camera.TriggerActivation.SetValue( Basler_GigECameraParams::TriggerActivation_RisingEdge );
            if( frame_trigger == "encoder" )
            {
                // todo: make configurable
                camera.ShaftEncoderModuleLineSelector.SetValue( Basler_GigECameraParams::ShaftEncoderModuleLineSelector_PhaseA );
                camera.ShaftEncoderModuleLineSource.SetValue( Basler_GigECameraParams::ShaftEncoderModuleLineSource_Line1 );
                camera.ShaftEncoderModuleLineSelector.SetValue( Basler_GigECameraParams::ShaftEncoderModuleLineSelector_PhaseB );
                camera.ShaftEncoderModuleLineSource.SetValue( Basler_GigECameraParams::ShaftEncoderModuleLineSource_Line2 );
                camera.ShaftEncoderModuleCounterMode.SetValue( Basler_GigECameraParams::ShaftEncoderModuleCounterMode_FollowDirection );
                camera.ShaftEncoderModuleMode.SetValue( Basler_GigECameraParams::ShaftEncoderModuleMode_ForwardOnly );
                camera.ShaftEncoderModuleCounterMax.SetValue( encoder_ticks - 1 );
                /// @todo compensate for mechanical jitter, if needed
                ///       see Runner_Users_manual.pdf, 8.3, Case 2
                camera.ShaftEncoderModuleReverseCounterMax.SetValue( 0 );
                camera.ShaftEncoderModuleCounterReset.Execute();
                camera.ShaftEncoderModuleReverseCounterReset.Execute();
            }
        }
    }
    GenApi::IEnumEntry* lineStart = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_LineStart );
    if( lineStart && GenApi::IsAvailable( lineStart ) )
    {
        std::string line_trigger = options.value< std::string >( "--line-trigger", "" );
        if( line_trigger.empty() )
        {
            camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_LineStart );
            camera.TriggerMode.SetValue( Basler_GigECameraParams::TriggerMode_Off );
        }
        else
        {
            camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_LineStart );
            camera.TriggerMode.SetValue( Basler_GigECameraParams::TriggerMode_On );
            Basler_GigECameraParams::TriggerSourceEnums t;
            if( line_trigger == "line1" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_Line1 ); }
            else if( line_trigger == "line2" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_Line2 ); }
            else if( line_trigger == "line3" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_Line3 ); }
            else if( line_trigger == "encoder" ) { camera.TriggerSource.SetValue( Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut ); }
            else { std::cerr << "basler-cat: line trigger '" << line_trigger << "' not implemented or invalid" << std::endl; return 1; }
            camera.TriggerActivation.SetValue( Basler_GigECameraParams::TriggerActivation_RisingEdge );
            camera.TriggerSelector.SetValue( Basler_GigECameraParams::TriggerSelector_LineStart );
            camera.TriggerMode.SetValue( Basler_GigECameraParams::TriggerMode_On );
            camera.TriggerActivation.SetValue( Basler_GigECameraParams::TriggerActivation_RisingEdge );
        }
    }
    if( chunk_mode )
    {
        std::cerr << "basler-cat: setting chunk mode..." << std::endl;
        if( !GenApi::IsWritable( camera.ChunkModeActive ) ) { std::cerr << "basler-cat: camera does not support chunk features" << std::endl; camera.Close(); return 1; }
        camera.ChunkModeActive.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_Framecounter );
        camera.ChunkEnable.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_Timestamp );
        camera.ChunkEnable.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_LineTriggerIgnoredCounter );
        camera.ChunkEnable.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_FrameTriggerIgnoredCounter );
        camera.ChunkEnable.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_LineTriggerEndToEndCounter );
        camera.ChunkEnable.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_FrameTriggerCounter );
        camera.ChunkEnable.SetValue( true );
        camera.ChunkSelector.SetValue( Basler_GigECameraParams::ChunkSelector_FramesPerTriggerCounter );
        camera.ChunkEnable.SetValue( true );
        parser = camera.CreateChunkParser();
        if( !parser ) { std::cerr << "basler-cat: failed to create chunk parser" << std::endl; camera.Close(); return 1; }
        std::cerr << "basler-cat: set chunk mode" << std::endl;
    }
    camera.ExposureMode.SetValue( Basler_GigECameraParams::ExposureMode_Timed );
    if( options.exists( "--exposure" )) { camera.ExposureTimeRaw.SetValue( options.value< unsigned int >( "--exposure" )); } // todo? auto exposure (see ExposureAutoEnums)
    if( options.exists( "--gain" ))
    {
        unsigned int gain = options.value< unsigned int >( "--gain" );
        if(verbose) { std::cerr<<"basler-cat: setting gain="<<gain<<std::endl; }
        camera.GainSelector.SetValue( Basler_GigECameraParams::GainSelector_All );
        camera.GainRaw.SetValue( gain );
        if( channels == 3 ) // todo: make configurable; also is not setting all not enough?
        {
            camera.GainSelector.SetValue( Basler_GigECameraParams::GainSelector_Red );
            camera.GainRaw.SetValue( gain );
            camera.GainSelector.SetValue( Basler_GigECameraParams::GainSelector_Green );
            camera.GainRaw.SetValue( gain );
            camera.GainSelector.SetValue( Basler_GigECameraParams::GainSelector_Blue );
            camera.GainRaw.SetValue( gain );
        }
    }
    if( options.exists( "--line-rate" )) { camera.AcquisitionLineRateAbs.SetValue( options.value< unsigned int >( "--line-rate" )); }
    if( options.exists( "--test-colour" )) { camera.TestImageSelector.SetValue( Basler_GigECameraParams::TestImageSelector_Testimage6 ); }
    else { camera.TestImageSelector.SetValue( Basler_GigECameraParams::TestImageSelector_Off ); }
    unsigned int payload_size = camera.PayloadSize.GetValue();
    if( verbose )
    { 
        std::cerr << "basler-cat: camera mtu size: " << camera.GevSCPSPacketSize.GetValue() << std::endl;
        std::cerr << "basler-cat: exposure: " << camera.ExposureTimeRaw.GetValue() << std::endl;
        std::cerr << "basler-cat: payload size: " << payload_size << std::endl;
    }
    std::vector< std::vector< char > > buffers( 2 ); // todo? make number of buffers configurable
    for( std::size_t i = 0; i < buffers.size(); ++i ) { buffers[i].resize( payload_size ); }
    grabber.MaxBufferSize.SetValue( buffers[0].size() );
    grabber.SocketBufferSize.SetValue( 127 );
    if( verbose )
    { 
        std::cerr << "basler-cat: socket buffer size: " << grabber.SocketBufferSize.GetValue() << std::endl;
        std::cerr << "basler-cat: max buffer size: " << grabber.MaxBufferSize.GetValue() << std::endl;
    }
    grabber.MaxNumBuffer.SetValue( buffers.size() ); // todo: use --buffer value for number of buffered images
    grabber.PrepareGrab(); // image size now must not be changed until FinishGrab() is called.
    std::vector< Pylon::StreamBufferHandle > buffer_handles( buffers.size() );
    for( std::size_t i = 0; i < buffers.size(); ++i )
    { 
        buffer_handles[i] = grabber.RegisterBuffer( &buffers[i][0], buffers[i].size() );
        grabber.QueueBuffer( buffer_handles[i], NULL );
    }

    if( chunk_mode )
    {
        snark::tbb::bursty_reader< ChunkPair > read( boost::bind( &capture_< ChunkPair >, boost::ref( camera ), boost::ref( grabber ) ), discard );
        tbb::filter_t< ChunkPair, void > write( tbb::filter::serial_in_order, boost::bind( &write_, _1 ) );
        snark::tbb::bursty_pipeline< ChunkPair > pipeline;
        camera.AcquisitionMode.SetValue( Basler_GigECameraParams::AcquisitionMode_Continuous );
        camera.AcquisitionStart.Execute(); // continuous acquisition mode        
        if( verbose ) { std::cerr << "basler-cat: running in chunk mode..." << std::endl; }
        pipeline.run( read, write );
        if( verbose ) { std::cerr << "basler-cat: shutting down..." << std::endl; }
        camera.AcquisitionStop();
        camera.DestroyChunkParser( parser );
    }
    else
    {
        snark::cv_mat::serialization serialization( cv_mat_options );
        snark::tbb::bursty_reader< Pair > reader( boost::bind( &capture_< Pair >, boost::ref( camera ), boost::ref( grabber ) ), discard );
        snark::imaging::applications::pipeline pipeline( serialization, filters, reader );
        //camera.AcquisitionMode.SetValue( Basler_GigECameraParams::AcquisitionMode_Continuous );
        camera.AcquisitionStart.Execute(); // continuous acquisition mode        
        if( verbose ) { std::cerr << "basler-cat: running..." << std::endl; }
        pipeline.run();
        if( verbose ) { std::cerr << "basler-cat: shutting down..." << std::endl; }
        camera.AcquisitionStop();
    }
    if( verbose ) { std::cerr << "basler-cat: acquisition stopped" << std::endl; }
    is_shutdown = true;
    while( !done ) { boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 100 ) ); }
    grabber.FinishGrab();
    Pylon::GrabResult result;
    while( grabber.RetrieveResult( result ) ); // get all buffers back
    for( std::size_t i = 0; i < buffers.size(); ++i ) { grabber.DeregisterBuffer( buffer_handles[i] ); }
    grabber.Close();
    camera.Close();
    return 0;
}

} // namespace gige

namespace usb {

int main( const comma::command_line_options& options )
{
    typedef Pylon::CBaslerUsbCamera Camera_t;

    Pylon::PylonAutoInitTerm auto_init_term;
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();
    Pylon::ITransportLayer* transport_layer( Pylon::CTlFactory::GetInstance().CreateTl( Camera_t::DeviceClass() ));
    if( !transport_layer )
    { 
        std::cerr << "basler-cat: failed to create transport layer" << std::endl;
        std::cerr << "            most likely PYLON_ROOT and GENICAM_ROOT_V2_1 environment variables not set" << std::endl;
        std::cerr << "            point them to your pylon installation, e.g:" << std::endl;
        std::cerr << "            export PYLON_ROOT=/opt/pylon" << std::endl;
        std::cerr << "            export GENICAM_ROOT_V2_1=/opt/pylon/genicam" << std::endl;
        return 1;
    }
    if( options.exists( "--list-cameras" ))
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        for( unsigned int i = 0; i < devices.size(); ++i ) { std::cerr << devices[i].GetFullName() << std::endl; }
        return 0;
    }
    timeout = options.value< double >( "--timeout", default_timeout ) * 1000.0;
    Camera_t camera;
    if( options.exists( "--address" ))
    {
        Pylon::CBaslerUsbDeviceInfo info;
        info.SetSerialNumber( options.value< std::string >( "--address" ).c_str() );
        camera.Attach( factory.CreateDevice( info ) );
    }
    else
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        if( devices.empty() ) { std::cerr << "basler-cat: no camera found" << std::endl; return 1; }
        std::cerr << "basler-cat: will connect to the first of " << devices.size() << " found device(s):" << std::endl;
        for( unsigned int i = 0; i < devices.size(); ++i ) { std::cerr << "    " << devices[i].GetFullName() << std::endl; }
        camera.Attach( transport_layer->CreateDevice( devices[0] ) );
    }
    if( verbose ) { std::cerr << "basler-cat: initialized camera" << std::endl; }
    if( verbose ) { std::cerr << "basler-cat: opening camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << "..." << std::endl; }
    camera.Open();
    if( verbose ) { std::cerr << "basler-cat: opened camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << std::endl; }
    Camera_t::StreamGrabber_t grabber( camera.GetStreamGrabber( 0 ) );
    grabber.Open();
    unsigned int channels;
    switch( cv_mat_options.get_header().type ) // quick and dirty
    {
        case CV_8UC1:
            channels = set_pixel_format_( camera, Basler_UsbCameraParams::PixelFormat_Mono8 );
            break;
        default:
            std::cerr << "basler-cat: type \"" << cv_mat_options.type << "\" not implemented or not supported by camera" << std::endl;
            return 1;
    }
    unsigned int max_width = camera.Width.GetMax();
    double offset_x = options.value< double >( "--offset-x", 0 );
    if( offset_x >= max_width ) { std::cerr << "basler-cat: expected --offset-x less than " << max_width << ", got " << offset_x << std::endl; return 1; }
    camera.OffsetX.SetValue( offset_x );
    unsigned int width = options.value< unsigned int >( "--width", max_width );
    width = ( ( unsigned long long )( offset_x ) + width ) < max_width ? width : max_width - offset_x;
    camera.Width.SetValue( width );
    unsigned int max_height = camera.Height.GetMax();
    //if( height < 512 ) { std::cerr << "basler-cat: expected height greater than 512, got " << height << std::endl; return 1; }
        
    // todo: is the colour line 2098 * 3 or ( 2098 / 3 ) * 3 ?
    //offset_y *= channels;
    //height *= channels;

    double offset_y = options.value< double >( "--offset-y", 0 );
    if( offset_y >= max_height ) { std::cerr << "basler-cat: expected --offset-y less than " << max_height << ", got " << offset_y << std::endl; return 1; }
    camera.OffsetY.SetValue( offset_y );
    unsigned int height = options.value< unsigned int >( "--height", max_height );
    height = ( ( unsigned long long )( offset_y ) + height ) < max_height ? height : ( max_height - offset_y );
    camera.Height.SetValue( height );
    if( verbose ) { std::cerr << "basler-cat: set width,height to " << width << "," << height << std::endl; }
    // todo: giving up... the commented code throws, but failure to stop acquisition, if active
    //       seems to lead to the following scenario:
    //       - power-cycle camera
    //       - view colour images: it works
    //       - view grey-scale images: it works
    //       - view colour images: it still displays grey-scale
    //if( verbose ) { std::cerr << "basler-cat: getting acquisition status... (frigging voodoo...)" << std::endl; }
    //GenApi::IEnumEntry* acquisition_status = camera.AcquisitionStatusSelector.GetEntry( Basler_UsbCameraParams::AcquisitionStatusSelector_AcquisitionActive );
    //if( acquisition_status && GenApi::IsAvailable( acquisition_status ) && camera.AcquisitionStatus() )
    //{
    //    if( verbose ) { std::cerr << "basler-cat: stopping acquisition..." << std::endl; }
    //    camera.AcquisitionStop.Execute();
    //    if( verbose ) { std::cerr << "basler-cat: acquisition stopped" << std::endl; }
    //}
        
    // todo: a hack for now
    camera.ExposureMode.SetValue( Basler_UsbCameraParams::ExposureMode_Timed );
    if( options.exists( "--exposure" )) { camera.ExposureTime.SetValue( options.value< unsigned int >( "--exposure" )); }
    else { camera.ExposureAuto.SetValue( Basler_UsbCameraParams::ExposureAuto_Once ); }
    if( options.exists( "--gain" ) )
    {
        unsigned int gain = options.value< unsigned int >( "--gain" );
        if(verbose) { std::cerr<<"basler-cat: setting gain="<<gain<<std::endl; }
        camera.GainSelector.SetValue( Basler_UsbCameraParams::GainSelector_All );
        camera.Gain.SetValue( gain );
    }
    if( options.exists( "--test-colour" )) { camera.TestImageSelector.SetValue( Basler_UsbCameraParams::TestImageSelector_Testimage6 ); }
    else { camera.TestImageSelector.SetValue( Basler_UsbCameraParams::TestImageSelector_Off ); }
    unsigned int payload_size = camera.PayloadSize.GetValue();
    if( verbose )
    { 
        std::cerr << "basler-cat: exposure: " << camera.ExposureTime.GetValue() << std::endl;
        std::cerr << "basler-cat: payload size: " << payload_size << std::endl;
    }
    std::vector< std::vector< char > > buffers( 2 ); // todo? make number of buffers configurable
    for( std::size_t i = 0; i < buffers.size(); ++i ) { buffers[i].resize( payload_size ); }
    grabber.MaxBufferSize.SetValue( buffers[0].size() );
    if( verbose )
    { 
        std::cerr << "basler-cat: max buffer size: " << grabber.MaxBufferSize.GetValue() << std::endl;
    }
    grabber.MaxNumBuffer.SetValue( buffers.size() ); // todo: use --buffer value for number of buffered images
    grabber.PrepareGrab(); // image size now must not be changed until FinishGrab() is called.
    std::vector< Pylon::StreamBufferHandle > buffer_handles( buffers.size() );
    for( std::size_t i = 0; i < buffers.size(); ++i )
    { 
        buffer_handles[i] = grabber.RegisterBuffer( &buffers[i][0], buffers[i].size() );
        grabber.QueueBuffer( buffer_handles[i], NULL );
    }

    if( chunk_mode )
    {
    }
    else
    {
        snark::cv_mat::serialization serialization( cv_mat_options );
        snark::tbb::bursty_reader< Pair > reader( boost::bind( &capture_< Pair >, boost::ref( camera ), boost::ref( grabber ) ), discard );
        snark::imaging::applications::pipeline pipeline( serialization, filters, reader );
        //camera.AcquisitionMode.SetValue( Basler_UsbCameraParams::AcquisitionMode_Continuous );
        camera.AcquisitionStart.Execute(); // continuous acquisition mode        
        if( verbose ) { std::cerr << "basler-cat: running..." << std::endl; }
        pipeline.run();
        if( verbose ) { std::cerr << "basler-cat: shutting down..." << std::endl; }
        camera.AcquisitionStop();
    }
    if( verbose ) { std::cerr << "basler-cat: acquisition stopped" << std::endl; }
    is_shutdown = true;
    while( !done ) { boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 100 ) ); }
    grabber.FinishGrab();
    Pylon::GrabResult result;
    while( grabber.RetrieveResult( result ) ); // get all buffers back
    for( std::size_t i = 0; i < buffers.size(); ++i ) { grabber.DeregisterBuffer( buffer_handles[i] ); }
    grabber.Close();
    camera.Close();
    return 0;
}

} // namespace usb

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    ::setenv( "PYLON_ROOT", STRINGIZED( BASLER_PYLON_DIR ), 0 );
    ::setenv( "GENICAM_ROOT_V2_1", STRINGIZED( BASLER_PYLON_GENICAM_DIR ), 0 );
    try
    {
        comma::command_line_options options( argc, argv, usage );

        verbose = options.exists( "--verbose" );
        if( verbose )
        {
            std::cerr << "basler-cat: PYLON_ROOT=" << ::getenv( "PYLON_ROOT" ) << std::endl;
            std::cerr << "basler-cat: GENICAM_ROOT_V2_1=" << ::getenv( "GENICAM_ROOT_V2_1" ) << std::endl;
            std::cerr << "basler-cat: initializing camera..." << std::endl;
        }

        filters = comma::join( options.unnamed( "--help,-h,--verbose,-v,--discard,--list-cameras,--header-only,--no-header,--test-colour", "-.*" ), ';' );
        cv_mat_options.header_only = options.exists( "--header-only" );
        cv_mat_options.no_header = options.exists( "--no-header" );
        csv = comma::csv::options( argc, argv );

        chunk_mode =    csv.has_field( "counters" ) // quick and dirty
                     || csv.has_field( "adjusted-t" )
                     || csv.has_field( "line" )
                     || csv.has_field( "line-count" )
                     || csv.has_field( "ticks" )
                     || csv.has_field( "counters/adjusted-t" )
                     || csv.has_field( "counters/line" )
                     || csv.has_field( "counters/line-count" )
                     || csv.has_field( "counters/ticks" );
        if( chunk_mode )
        {
            if( !options.exists( "--encoder-ticks" )) { std::cerr << "basler-cat: chunk mode, please specify --encoder-ticks" << std::endl; return 1; }
            if( !filters.empty() ) { std::cerr << "basler-cat: chunk mode, cannot handle filters; use: basler-cat | cv-cat <filters> instead" << std::endl; return 1; }
            unsigned int height = options.value< unsigned int >( "--height", 1 );
            if( height != 1 ) { std::cerr << "basler-cat: only --height=1 implemented in chunk mode" << std::endl; return 1; }
            std::vector< std::string > v = comma::split( csv.fields, ',' );
            std::string format;
            for( unsigned int i = 0; i < v.size(); ++i )
            {
                if( v[i] == "t" ) { v[i] = "header/" + v[i]; format += "t"; }
                else if( v[i] == "rows" || v[i] == "cols" || v[i] == "size" || v[i] == "type" ) { v[i] = "header/" + v[i]; format += "ui"; }
                else if( v[i] == "adjusted-t" ) { v[i] = "counters/" + v[i]; format += "t"; }
                else if( v[i] == "line-count" || v[i] == "ticks" ) { v[i] = "counters/" + v[i]; format += "ul"; }
                else if( v[i] == "line" ) { v[i] = "counters/" + v[i]; format += "ui"; }
                else { std::cerr << "basler-cat: expected field, got '" << v[i] << "'" << std::endl; return 1; }
            }
            csv.fields = comma::join( v, ',' );
            csv.full_xpath = true;
            csv.format( format );
        }
        if( !options.exists( "--buffer" ) && options.exists( "--discard" )) { discard = 1; }

        bool is_gige = ( options.value< std::string >( "--camera-type", "gige" ) == "gige" );
        int return_value;
        if( is_gige ) { return_value = gige::main( options ); }
        else { return_value = usb::main( options ); }
        if( verbose ) { std::cerr << "basler-cat: done" << std::endl; }
        return return_value;
    }
#if ( PYLON_VERSION_MAJOR >= 5 )
    catch(const Pylon::GenericException& e) { std::cerr << "basler-cat: pylon exception: " << e.what() << std::endl; }
#endif
    catch( std::exception& ex ) { std::cerr << "basler-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "basler-cat: unknown exception" << std::endl; }
    return 1;
}
