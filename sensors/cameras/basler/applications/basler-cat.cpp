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

#include <boost/regex.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../../../../imaging/cv_mat/pipeline.h"
#include "../../../../imaging/cv_mat/bursty_pipeline.h"
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <pylon/usb/BaslerUsbCamera.h>

static double default_timeout = 3.0;
static const char* possible_header_fields = "t,rows,cols,type,size,counters";
static const char* default_header_fields = "t,rows,cols,type";

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --address --list-cameras"
        " --discard --buffer"
        " --fields -f"
        " --image-type"
        " --offset-x --offset-y --width --height"
        " --frame-trigger --line-trigger --line-rate"
        " --encoder-ticks"
        " --header-only --no-header"
        " --packet-size"
        " --exposure --gain"
        " --timeout"
        " --test-colour"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nAcquire images from a basler camera";
    std::cerr << "\nOutput to stdout as serialized cv::Mat";
    std::cerr << "\n";
    std::cerr << "\nUsage: basler-cat [<options>] [<filters>]";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h                 display help message";
    std::cerr << "\n    --address=[<address>]     camera address; default: first available";
    std::cerr << "\n    --discard                 discard frames, if cannot keep up;";
    std::cerr << "\n                              same as --buffer=1 (which is not a great setting)";
    std::cerr << "\n    --buffer=[<buffers>]      maximum buffer size before discarding frames";
    std::cerr << "\n                              default: unlimited";
    std::cerr << "\n    --list-cameras            output camera list and exit";
    std::cerr << "\n                              add --verbose for more detail";
    std::cerr << "\n    --fields,-f=[<fields>]    header fields, possible values:";
    std::cerr << "\n                              possible values: " << possible_header_fields;
    std::cerr << "\n                              default: " << default_header_fields;
    std::cerr << "\n    --image-type=[<type>]     image type; default: 3ub; --verbose for more";
    std::cerr << "\n    --offset-x=[<pixels>]     offset in pixels in the line";
    std::cerr << "\n    --offset-y=[<pixels>]     offset in lines in the frame";
    std::cerr << "\n    --width=[<pixels>]        line width in pixels; default: max";
    std::cerr << "\n    --height=[<pixels>]       number of lines in frame (in chunk mode always 1)";
    std::cerr << "\n                              default: max";
    std::cerr << "\n    --frame-trigger=[<type>]  'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-trigger=[<type>]   'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-rate=[<num>]       line acquisition rate";
    std::cerr << "\n    --encoder-ticks=[<num>]   number of encoder ticks until the count resets";
    std::cerr << "\n                              (reused for line number in frame in chunk mode)";
    std::cerr << "\n    --header-only             output header only";
    std::cerr << "\n    --no-header               output image data only";
    std::cerr << "\n    --packet-size=[<bytes>]   mtu size on camera side, should not be larger ";
    std::cerr << "\n                              than your lan and network interface";
    std::cerr << "\n    --exposure=[<num>]        exposure";
    std::cerr << "\n    --gain=[<num>]            gain";
    std::cerr << "\n    --timeout=[<seconds>]     frame acquisition timeout; default " << default_timeout << "s";
    std::cerr << "\n    --test-colour             output colour test image";
    std::cerr << "\n    --verbose,-v              be more verbose";
    std::cerr << "\n";
    std::cerr << "\nFor GigE cameras <address> is the device ip address, for USB cameras it is";
    std::cerr << "\nthe USB address. Both can be determined by --list-cameras --verbose.";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\n";
        std::cerr << snark::cv_mat::filters::usage();
        std::cerr << snark::cv_mat::serialization::options::type_usage();
    }
    std::cerr << "\nNote: there is a glitch or a subtle feature in basler line camera:";
    std::cerr << "\n    - power-cycle camera";
    std::cerr << "\n    - view colour images: it works";
    std::cerr << "\n    - view grey-scale images: it works";
    std::cerr << "\n    - view colour images: it still displays grey-scale";
    std::cerr << "\n";
    std::cerr << "\n    Even in their native viewer you need to set colour image repeatedly and";
    std::cerr << "\n    with pure luck it works, but we have not managed to do it in software.";
    std::cerr << "\n    The remedy: power-cycle the camera";
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

template < typename T >
static void set_( boost::posix_time::ptime& timestamp
                , boost::posix_time::ptime& t
                , const Pylon::GrabResult&
                , T& )
{
    timestamp = t;
}

template < typename T >
static void set_( ChunkData& d
                , boost::posix_time::ptime& t
                , const Pylon::GrabResult& result
                , T& camera )
{
    parser->AttachBuffer( ( unsigned char* ) result.Buffer(), result.GetPayloadSize() );
    d.timestamp = t;
    d.frames = camera.ChunkFramecounter();
    d.ticks = camera.ChunkTimestamp();
    d.line_trigger_ignored = camera.ChunkLineTriggerIgnoredCounter();
    d.frame_trigger_ignored = camera.ChunkFrameTriggerIgnoredCounter();
    d.line_trigger_end_to_end = camera.ChunkLineTriggerEndToEndCounter();
    d.frame_trigger = camera.ChunkFrameTriggerCounter();
    d.frames_per_trigger = camera.ChunkFramesPerTriggerCounter();
    parser->DetachBuffer();
}

template < typename T, typename P >
static P capture_( T& camera, typename T::StreamGrabber_t& grabber )
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
                std::cerr << "basler-cat: cv::mat type " << header.type << " not supported" << std::endl;
        }
        set_< T >( pair.first, t, result, camera );
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

struct pixel_format_desc
{
    std::string name;
    unsigned int channels;
};

// Each transport supports a different set of PixelFormatEnums

pixel_format_desc pixel_format_to_desc( Basler_GigECameraParams::PixelFormatEnums pixel_format )
{
    pixel_format_desc pf;
    switch( pixel_format )
    {
        case Basler_GigECameraParams::PixelFormat_Mono8: pf.name = "Mono8"; pf.channels = 1; break;
        case Basler_GigECameraParams::PixelFormat_RGB8Packed: pf.name = "RGB8Packed"; pf.channels = 3; break;
        default: COMMA_THROW( comma::exception, "pixel format " << pixel_format << " not implemented" );
    }
    return pf;
}

pixel_format_desc pixel_format_to_desc( Basler_UsbCameraParams::PixelFormatEnums pixel_format )
{
    pixel_format_desc pf;
    switch( pixel_format )
    {
        case Basler_UsbCameraParams::PixelFormat_Mono8: pf.name = "Mono8"; pf.channels = 1; break;
        default: COMMA_THROW( comma::exception, "pixel format " << pixel_format << " not implemented" );
    }
    return pf;
}

template < typename T, typename P >
static unsigned int set_pixel_format_( T& camera, P type )
{
    pixel_format_desc pixel_format = pixel_format_to_desc( type );

    if( camera.PixelFormat() == type ) { return pixel_format.channels; }
    GenApi::NodeList_t entries;
    camera.PixelFormat.GetEntries( entries );
    bool supported = false;
    comma::verbose << "supported pixel format(s): ";
    for( std::size_t i = 0; i < entries.size(); ++i )
    {
        GenApi::INode* node = entries[i]; // bloody voodoo
        if( !IsAvailable( node->GetAccessMode() ) ) { continue; }
        GenApi::IEnumEntry* e = dynamic_cast< GenApi::IEnumEntry* >( node );
        if( comma::verbose ) { std::cerr << ( i > 0 ? ", " : "" ) << e->GetSymbolic(); }
        supported = supported || pixel_format.name != e->GetSymbolic().c_str();
    }
    if( comma::verbose ) { std::cerr << std::endl; }
    if( !supported ) { COMMA_THROW( comma::exception, "pixel format " << type << " is not supported" ); }
    comma::verbose << "setting pixel format..." << std::endl;
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
        camera.PixelFormat = type;
        boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 10 ) );
    }
    if( camera.PixelFormat() != type ) { COMMA_THROW( comma::exception, "failed to set pixel format after " << retries << " attempts; try again or power-cycle the camera" ); }
    comma::verbose << "pixel format set to " << pixel_format.name << std::endl;
    return pixel_format.channels;
}

unsigned int num_channels( Pylon::CBaslerGigECamera& camera, comma::uint32 type )
{
    switch( type )
    {
        case CV_8UC1:
            return set_pixel_format_< Pylon::CBaslerGigECamera, Basler_GigECameraParams::PixelFormatEnums >( camera, Basler_GigECameraParams::PixelFormat_Mono8 );
        case CV_8UC3:
            return set_pixel_format_< Pylon::CBaslerGigECamera, Basler_GigECameraParams::PixelFormatEnums >( camera, Basler_GigECameraParams::PixelFormat_RGB8Packed );
        default:
            COMMA_THROW( comma::exception, "type \"" << type << "\" not implemented or not supported by camera" );
    }
}

unsigned int num_channels( Pylon::CBaslerUsbCamera& camera, comma::uint32 type )
{
    switch( type )
    {
        case CV_8UC1:
            return set_pixel_format_< Pylon::CBaslerUsbCamera, Basler_UsbCameraParams::PixelFormatEnums >( camera, Basler_UsbCameraParams::PixelFormat_Mono8 );
        default:
            COMMA_THROW( comma::exception, "type \"" << type << "\" not implemented or not supported by camera" );
    }
}

void list_cameras()
{
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t devices;
    factory.EnumerateDevices( devices );
    Pylon::DeviceInfoList_t::const_iterator it;
    for( it = devices.begin(); it != devices.end(); ++it )
    {
        if( comma::verbose )
        {
            std::cerr << "\nVendor:     " << it->GetVendorName();
            std::cerr << "\nModel:      " << it->GetModelName();
            std::cerr << "\nVersion:    " << it->GetDeviceVersion();
            std::cerr << "\nType:       " << it->GetDeviceClass();
            std::cerr << "\nSerial no.: " << it->GetSerialNumber();
            std::cerr << "\nFull name:  " << it->GetFullName();
            std::cerr << std::endl;
        }
        else { std::cerr << it->GetFullName() << std::endl; }
    }
    if( comma::verbose && !devices.empty() ) { std::cerr << std::endl; }
}

bool is_ip_address( std::string str )
{
    boost::regex ip_regex( "[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+" ); // somewhat naive
    return boost::regex_match( str, ip_regex );
}

Pylon::IPylonDevice* create_device( const std::string& address )
{
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();

    if( address.empty() )
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        if( devices.empty() ) { std::cerr << "basler-cat: no camera found" << std::endl; return NULL; }
        std::cerr << "basler-cat: will connect to the first of " << devices.size()
                  << " found device(s):" << std::endl;
        Pylon::DeviceInfoList_t::const_iterator it;
        for( it = devices.begin(); it != devices.end(); ++it )
        {
            std::cerr << "    " << it->GetFullName() << std::endl;
        }
        return factory.CreateDevice( devices[0] );
    }
    else
    {
        if( is_ip_address( address ))
        {
            Pylon::CBaslerGigEDeviceInfo gige_device_info;
            gige_device_info.SetIpAddress( address.c_str() );
            return factory.CreateDevice( gige_device_info );
        }
        else
        {
            Pylon::CDeviceInfo device_info;
            device_info.SetFullName( address.c_str() );
            return factory.CreateDevice( device_info );
        }
    }
}

static bool chunk_mode = false;
static std::string filters;

bool configure_trigger( Pylon::CBaslerUsbCamera& camera, const comma::command_line_options& options )
{
    return true;
}

bool configure_trigger( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
{
    GenApi::IEnumEntry* acquisitionStart = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_AcquisitionStart );
    std::string frame_trigger = options.value< std::string >( "--frame-trigger", "" );
    if( acquisitionStart && GenApi::IsAvailable( acquisitionStart ) )
    {
        camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_AcquisitionStart;
        camera.TriggerMode = ( frame_trigger.empty() ? Basler_GigECameraParams::TriggerMode_Off : Basler_GigECameraParams::TriggerMode_On );
    }
    GenApi::IEnumEntry* frameStart = camera.TriggerSelector.GetEntry( Basler_GigECameraParams::TriggerSelector_FrameStart );
    if( frameStart && GenApi::IsAvailable( frameStart ) )
    {
        //if( frame_trigger.empty() ) { frame_trigger = line_trigger; }
        if( frame_trigger.empty() )
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_FrameStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_Off;
        }
        else
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_FrameStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            Basler_GigECameraParams::TriggerSourceEnums t;
            if( frame_trigger == "line1" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Line1; }
            if( frame_trigger == "line2" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Line2; }
            if( frame_trigger == "line3" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Line3; }
            else if( frame_trigger == "encoder" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut; }
            else { std::cerr << "basler-cat: frame trigger '" << frame_trigger << "' not implemented or invalid" << std::endl; return false; }
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
            if( frame_trigger == "encoder" )
            {
                // todo: make configurable
                camera.ShaftEncoderModuleLineSelector = Basler_GigECameraParams::ShaftEncoderModuleLineSelector_PhaseA;
                camera.ShaftEncoderModuleLineSource = Basler_GigECameraParams::ShaftEncoderModuleLineSource_Line1;
                camera.ShaftEncoderModuleLineSelector = Basler_GigECameraParams::ShaftEncoderModuleLineSelector_PhaseB;
                camera.ShaftEncoderModuleLineSource = Basler_GigECameraParams::ShaftEncoderModuleLineSource_Line2;
                camera.ShaftEncoderModuleCounterMode = Basler_GigECameraParams::ShaftEncoderModuleCounterMode_FollowDirection;
                camera.ShaftEncoderModuleMode = Basler_GigECameraParams::ShaftEncoderModuleMode_ForwardOnly;
                camera.ShaftEncoderModuleCounterMax = encoder_ticks - 1;
                /// @todo compensate for mechanical jitter, if needed
                ///       see Runner_Users_manual.pdf, 8.3, Case 2
                camera.ShaftEncoderModuleReverseCounterMax = 0;
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
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_Off;
        }
        else
        {
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            Basler_GigECameraParams::TriggerSourceEnums t;
            if( line_trigger == "line1" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Line1; }
            else if( line_trigger == "line2" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Line2; }
            else if( line_trigger == "line3" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_Line3; }
            else if( line_trigger == "encoder" ) { camera.TriggerSource = Basler_GigECameraParams::TriggerSource_ShaftEncoderModuleOut; }
            else { std::cerr << "basler-cat: line trigger '" << line_trigger << "' not implemented or invalid" << std::endl; return false; }
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
            camera.TriggerSelector = Basler_GigECameraParams::TriggerSelector_LineStart;
            camera.TriggerMode = Basler_GigECameraParams::TriggerMode_On;
            camera.TriggerActivation = Basler_GigECameraParams::TriggerActivation_RisingEdge;
        }
    }
    return true;
}

void configure_chunk_mode( Pylon::CBaslerUsbCamera& camera )
{
    COMMA_THROW( comma::exception, "chunk mode not supported for USB cameras" );
}

void configure_chunk_mode( Pylon::CBaslerGigECamera& camera )
{
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_Framecounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_Timestamp;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_LineTriggerIgnoredCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_FrameTriggerIgnoredCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_LineTriggerEndToEndCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_FrameTriggerCounter;
    camera.ChunkEnable = true;
    camera.ChunkSelector = Basler_GigECameraParams::ChunkSelector_FramesPerTriggerCounter;
    camera.ChunkEnable = true;
}

void set_exposure( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
{
    camera.ExposureMode = Basler_GigECameraParams::ExposureMode_Timed;
    if( options.exists( "--exposure" )) { camera.ExposureTimeRaw = ( options.value< unsigned int >( "--exposure" )); } // todo? auto exposure (see ExposureAutoEnums)
}

void set_exposure( Pylon::CBaslerUsbCamera& camera, const comma::command_line_options& options )
{
    camera.ExposureMode = Basler_UsbCameraParams::ExposureMode_Timed;
    if( options.exists( "--exposure" )) { camera.ExposureTime = ( options.value< unsigned int >( "--exposure" )); }
    else { camera.ExposureAuto = Basler_UsbCameraParams::ExposureAuto_Once; }
}

void set_gain( Pylon::CBaslerGigECamera& camera, unsigned int gain, unsigned int channels )
{
    camera.GainSelector = Basler_GigECameraParams::GainSelector_All;
    camera.GainRaw = gain;
    if( channels == 3 ) // todo: make configurable; also is not setting all not enough?
    {
        camera.GainSelector = Basler_GigECameraParams::GainSelector_Red;
        camera.GainRaw = gain;
        camera.GainSelector = Basler_GigECameraParams::GainSelector_Green;
        camera.GainRaw = gain;
        camera.GainSelector = Basler_GigECameraParams::GainSelector_Blue;
        camera.GainRaw = gain;
    }
}

void set_gain( Pylon::CBaslerUsbCamera& camera, unsigned int gain, unsigned int )
{
    camera.GainSelector = Basler_UsbCameraParams::GainSelector_All;
    camera.Gain = gain;
}

void set_line_rate( Pylon::CBaslerGigECamera& camera, unsigned int line_rate )
{
    camera.AcquisitionLineRateAbs = line_rate;
}

void set_line_rate( Pylon::CBaslerUsbCamera& camera, unsigned int )
{
    COMMA_THROW( comma::exception, "--line-rate not supported for USB cameras" );
}

void set_packet_size( Pylon::CBaslerGigECamera& camera, unsigned int packet_size )
{
    camera.GevSCPSPacketSize = packet_size;
}

void set_packet_size( Pylon::CBaslerUsbCamera& camera, unsigned int )
{
    COMMA_THROW( comma::exception, "--packet-size not supported for USB cameras" );
}

void set_socket_buffer_size( Pylon::CBaslerGigECamera::StreamGrabber_t& grabber, unsigned int socket_buffer_size )
{
    grabber.SocketBufferSize = socket_buffer_size;
}

void set_socket_buffer_size( Pylon::CBaslerUsbCamera::StreamGrabber_t&, unsigned int ) {}

void set_test_image( Pylon::CBaslerGigECamera& camera, bool on )
{
    if( on ) { camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage6; }
    else { camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Off; }
}

void set_test_image( Pylon::CBaslerUsbCamera& camera, bool on )
{
    if( on ) { camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage6; }
    else { camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Off; }
}

void show_config( Pylon::CBaslerGigECamera& camera )
{
    comma::verbose << "camera mtu size: " << camera.GevSCPSPacketSize() << std::endl;
    comma::verbose << "exposure: " << camera.ExposureTimeRaw() << std::endl;
    comma::verbose << "payload size: " << camera.PayloadSize() << std::endl;
}

void show_config( Pylon::CBaslerUsbCamera& camera )
{
    comma::verbose << "exposure: " << camera.ExposureTime() << std::endl;
    comma::verbose << "payload size: " << camera.PayloadSize() << std::endl;
}

void show_config( Pylon::CBaslerGigECamera::StreamGrabber_t& grabber )
{
    comma::verbose << "socket buffer size: " << grabber.SocketBufferSize() << std::endl;
    comma::verbose << "max buffer size: " << grabber.MaxBufferSize() << std::endl;
}

void show_config( Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber )
{
    comma::verbose << "max buffer size: " << grabber.MaxBufferSize() << std::endl;
}

void run_pipeline( Pylon::CBaslerGigECamera& camera
                 , Pylon::CBaslerGigECamera::StreamGrabber_t& grabber
                 , bool chunk_mode
                 , unsigned int max_queue_size
                 , unsigned int max_queue_capacity )
{
    if( chunk_mode )
    {
        snark::tbb::bursty_reader< ChunkPair > read( boost::bind( &capture_< Pylon::CBaslerGigECamera, ChunkPair >, boost::ref( camera ), boost::ref( grabber ) ), max_queue_size, max_queue_capacity );
        tbb::filter_t< ChunkPair, void > write( tbb::filter::serial_in_order, boost::bind( &write_, _1 ) );
        snark::tbb::bursty_pipeline< ChunkPair > pipeline;
        camera.AcquisitionMode = Basler_GigECameraParams::AcquisitionMode_Continuous;
        camera.AcquisitionStart.Execute(); // continuous acquisition mode
        comma::verbose << "running in chunk mode..." << std::endl;
        pipeline.run( read, write );
        comma::verbose << "shutting down..." << std::endl;
        camera.AcquisitionStop();
        camera.DestroyChunkParser( parser );
    }
    else
    {
        snark::cv_mat::serialization serialization( cv_mat_options );
        snark::tbb::bursty_reader< Pair > reader( boost::bind( &capture_< Pylon::CBaslerGigECamera, Pair >, boost::ref( camera ), boost::ref( grabber ) ), max_queue_size, max_queue_capacity );
        snark::imaging::applications::pipeline pipeline( serialization, filters, reader );
        //camera.AcquisitionMode = Basler_GigECameraParams::AcquisitionMode_Continuous;
        camera.AcquisitionStart.Execute(); // continuous acquisition mode
        comma::verbose << "running..." << std::endl;
        pipeline.run();
        comma::verbose << "shutting down..." << std::endl;
        camera.AcquisitionStop();
    }
}

void run_pipeline( Pylon::CBaslerUsbCamera& camera
                 , Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber
                 , bool chunk_mode
                 , unsigned int max_queue_size
                 , unsigned int max_queue_capacity )
{
    if( chunk_mode )
    {
        COMMA_THROW( comma::exception, "chunk mode not supported for USB cameras" );
    }
    else
    {
        snark::cv_mat::serialization serialization( cv_mat_options );
        snark::tbb::bursty_reader< Pair > reader( boost::bind( &capture_< Pylon::CBaslerUsbCamera, Pair >, boost::ref( camera ), boost::ref( grabber ) ), max_queue_size, max_queue_capacity );
        snark::imaging::applications::pipeline pipeline( serialization, filters, reader );
        //camera.AcquisitionMode = Basler_UsbCameraParams::AcquisitionMode_Continuous;
        camera.AcquisitionStart.Execute(); // continuous acquisition mode
        comma::verbose << "running..." << std::endl;
        pipeline.run();
        comma::verbose << "shutting down..." << std::endl;
        camera.AcquisitionStop();
    }
}

template< typename T >
int run( T& camera, const comma::command_line_options& options )
{
    typedef T camera_t;

    timeout = options.value< double >( "--timeout", default_timeout ) * 1000.0;
    comma::verbose << "initialized camera" << std::endl;
    comma::verbose << "opening camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << "..." << std::endl;
    camera.Open();
    comma::verbose << "opened camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << std::endl;
    typename camera_t::StreamGrabber_t grabber( camera.GetStreamGrabber( 0 ) );
    grabber.Open();
    unsigned int channels = num_channels( camera, cv_mat_options.get_header().type );
    unsigned int max_width = camera.Width.GetMax();
    double offset_x = options.value< double >( "--offset-x", 0 );
    if( offset_x >= max_width ) { std::cerr << "basler-cat: expected --offset-x less than " << max_width << ", got " << offset_x << std::endl; return 1; }
    camera.OffsetX = offset_x;
    unsigned int width = options.value< unsigned int >( "--width", max_width );
    width = ( ( unsigned long long )( offset_x ) + width ) < max_width ? width : max_width - offset_x;
    camera.Width = width;
    unsigned int max_height = camera.Height.GetMax();
    //if( height < 512 ) { std::cerr << "basler-cat: expected height greater than 512, got " << height << std::endl; return 1; }

    // todo: is the colour line 2098 * 3 or ( 2098 / 3 ) * 3 ?
    //offset_y *= channels;
    //height *= channels;

    double offset_y = options.value< double >( "--offset-y", 0 );
    if( offset_y >= max_height ) { std::cerr << "basler-cat: expected --offset-y less than " << max_height << ", got " << offset_y << std::endl; return 1; }
    camera.OffsetY = offset_y;
    unsigned int height = options.value< unsigned int >( "--height", max_height );
    height = ( ( unsigned long long )( offset_y ) + height ) < max_height ? height : ( max_height - offset_y );
    camera.Height = height;
    comma::verbose << "set width,height to " << width << "," << height << std::endl;
    if( options.exists( "--packet-size" )) { set_packet_size( camera, options.value< unsigned int >( "--packet-size" )); }
    // todo: giving up... the commented code throws, but failure to stop acquisition, if active
    //       seems to lead to the following scenario:
    //       - power-cycle camera
    //       - view colour images: it works
    //       - view grey-scale images: it works
    //       - view colour images: it still displays grey-scale
    //comma::verbose << "getting acquisition status... (frigging voodoo...)" << std::endl;
    //GenApi::IEnumEntry* acquisition_status = camera.AcquisitionStatusSelector.GetEntry( Basler_GigECameraParams::AcquisitionStatusSelector_AcquisitionActive );
    //if( acquisition_status && GenApi::IsAvailable( acquisition_status ) && camera.AcquisitionStatus() )
    //{
    //    comma::verbose << "stopping acquisition..." << std::endl;
    //    camera.AcquisitionStop.Execute();
    //    comma::verbose << "acquisition stopped" << std::endl;
    //}

    if( !configure_trigger( camera, options )) { return 1; }
    if( chunk_mode )
    {
        std::cerr << "basler-cat: setting chunk mode..." << std::endl;
        if( !GenApi::IsWritable( camera.ChunkModeActive )) { std::cerr << "basler-cat: camera does not support chunk features" << std::endl; camera.Close(); return 1; }
        camera.ChunkModeActive = true;
        configure_chunk_mode( camera );
        parser = camera.CreateChunkParser();
        if( !parser ) { std::cerr << "basler-cat: failed to create chunk parser" << std::endl; camera.Close(); return 1; }
        std::cerr << "basler-cat: set chunk mode" << std::endl;
    }
    set_exposure( camera, options );
    if( options.exists( "--gain" ))
    {
        unsigned int gain = options.value< unsigned int >( "--gain" );
        comma::verbose << "setting gain=" << gain << std::endl;
        set_gain( camera, gain, channels );
    }
    if( options.exists( "--line-rate" )) { set_line_rate( camera, options.value< unsigned int >( "--line-rate" )); }
    set_test_image( camera, options.exists( "--test-colour" ));
    show_config( camera );
    std::vector< std::vector< char > > buffers( 2 ); // todo? make number of buffers configurable
    for( std::size_t i = 0; i < buffers.size(); ++i ) { buffers[i].resize( camera.PayloadSize() ); }
    grabber.MaxBufferSize = buffers[0].size();
    set_socket_buffer_size( grabber, 127 );
    show_config( grabber );
    grabber.MaxNumBuffer = buffers.size(); // todo: use --buffer value for number of buffered images
    grabber.PrepareGrab(); // image size now must not be changed until FinishGrab() is called.
    std::vector< Pylon::StreamBufferHandle > buffer_handles( buffers.size() );
    for( std::size_t i = 0; i < buffers.size(); ++i )
    {
        buffer_handles[i] = grabber.RegisterBuffer( &buffers[i][0], buffers[i].size() );
        grabber.QueueBuffer( buffer_handles[i], NULL );
    }

    unsigned int max_queue_size = options.value< unsigned int >( "--buffer", options.exists( "--discard" ));

    run_pipeline( camera, grabber, chunk_mode, max_queue_size, max_queue_size * 3 );

    comma::verbose << "acquisition stopped" << std::endl;
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

#define QUOTED( arg ) #arg
#define STRINGIZED( arg ) QUOTED( arg )

int main( int argc, char** argv )
{
    ::setenv( "PYLON_ROOT", STRINGIZED( BASLER_PYLON_DIR ), 0 );
    ::setenv( "GENICAM_ROOT_V2_1", STRINGIZED( BASLER_PYLON_GENICAM_DIR ), 0 );
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );

        Pylon::PylonAutoInitTerm auto_init_term;

        if( options.exists( "--list-cameras" ))
        {
            list_cameras();
            return 0;
        }

        comma::verbose << "PYLON_ROOT=" << ::getenv( "PYLON_ROOT" ) << std::endl;
        comma::verbose << "GENICAM_ROOT_V2_1=" << ::getenv( "GENICAM_ROOT_V2_1" ) << std::endl;
        comma::verbose << "initializing camera..." << std::endl;

        std::string address = options.value< std::string >( "--address", "" );
        Pylon::IPylonDevice* device = create_device( address );
        if( !device )
        {
            std::cerr << "unable to open camera";
            if( !address.empty() ) { std::cerr << " for address " << address; }
            std::cerr << std::endl;
            return 1;
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

        int return_value = 1;
        Pylon::String_t device_class = device->GetDeviceInfo().GetDeviceClass();
        if ( device_class == "BaslerGigE" )
        {
            Pylon::CBaslerGigECamera camera;
            camera.Attach( device );
            return_value = run< Pylon::CBaslerGigECamera >( camera, options );
        }
        else if( device_class == "BaslerUsb" )
        {
            Pylon::CBaslerUsbCamera camera;
            camera.Attach( device );
            return_value = run< Pylon::CBaslerUsbCamera >( camera, options );
        }
        else
        {
            std::cerr << "basler-cat: unsupported device type of " << device_class << std::endl;
        }
        comma::verbose << "done" << std::endl;
        return return_value;
    }
#if ( PYLON_VERSION_MAJOR >= 5 )
    catch(const Pylon::GenericException& e) { std::cerr << "basler-cat: pylon exception: " << e.what() << std::endl; }
#endif
    catch( std::exception& ex ) { std::cerr << "basler-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "basler-cat: unknown exception" << std::endl; }
    return 1;
}
