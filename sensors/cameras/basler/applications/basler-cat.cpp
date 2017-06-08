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

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
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
        " --address --serial-number --list-cameras"
        " --test-image"
        " --fields -f"
        " --header-only --no-header"
        " --offset-x --offset-y --width --height"
        " --pixel-format"
        " --frame-rate --exposure --gain"
        " --discard --buffer"
        " --frame-trigger --line-trigger --line-rate"
        " --encoder-ticks"
        " --timeout"
        " --packet-size"
        ;

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nacquire images from a basler camera";
    std::cerr << "\n";
    std::cerr << "\noutput to stdout as serialized cv::Mat";
    std::cerr << "\n";
    std::cerr << "\nusage: basler-cat [<options>] [<filters>]";
    std::cerr << "\n";
    std::cerr << "\ngeneral options";
    std::cerr << "\n    --help,-h                    display help message";
    std::cerr << "\n    --address=[<address>]        camera address; default: first available";
    std::cerr << "\n    --serial-number=[<num>]      camera serial number, alternative to --address";
    std::cerr << "\n    --list-cameras               output camera list and exit";
    std::cerr << "\n                                 add --verbose for more detail";
    std::cerr << "\n    --test-image=[<num>]         output test image <num>; possible values: 1-6";
    std::cerr << "\n    --verbose,-v                 be more verbose";
    std::cerr << "\n";
    std::cerr << "\nimage options";
    std::cerr << "\n    --fields,-f=[<fields>]       header fields, possible values:";
    std::cerr << "\n                                 possible values: " << possible_header_fields;
    std::cerr << "\n                                 default: " << default_header_fields;
    std::cerr << "\n    --header-only                output header only";
    std::cerr << "\n    --no-header                  output image data only";
    std::cerr << "\n    --height=[<pixels>]          number of lines in frame (in chunk mode always 1)";
    std::cerr << "\n                                 default: max";
    std::cerr << "\n    --width=[<pixels>]           line width in pixels; default: max";
    std::cerr << "\n    --offset-x=[<pixels>]        offset in pixels in the line; todo: make sure it works on images with more than 1 channel";
    std::cerr << "\n    --offset-y=[<pixels>]        offset in lines in the frame; todo: make sure it works on images with more than 1 channel";
    std::cerr << "\n    --pixel-format=[<format>]    pixel format; lower case accepted; currently supported formats:";
    std::cerr << "\n                                     gige cameras";
    std::cerr << "\n                                         Mono8: stdout output format: ub";
    std::cerr << "\n                                         RGB8: stdout output format: 3ub";
    std::cerr << "\n                                     usb cameras";
    std::cerr << "\n                                         Mono8: stdout output format: ub";
    std::cerr << "\n                                         Mono10: stdout output format: uw";
    std::cerr << "\n                                         Mono10p: stdout output format: uw (packed format; todo)";
    std::cerr << "\n                                         Mono12: stdout output format: uw";
    std::cerr << "\n                                         Mono12p: stdout output format: 3ub (apply unpack12 to get 16-bit padded image)";
    std::cerr << "\n                                     default: use the current camera setting";
    std::cerr << "\n                                     format names correspond to enum found in basler header files";
    std::cerr << "\n    --reverse-x                  horizontal mirror image";
    std::cerr << "\n    --reverse-y                  vertical mirror image";
    std::cerr << "\n    --binning-horizontal=<pixels>[,<mode>]";
    std::cerr << "\n                                 sets the number of adjacent horizontal pixels to bin";
    std::cerr << "\n                                 <mode>: sum, average (default: sum)";
    std::cerr << "\n                                 ROI will refer to binned columns";
    std::cerr << "\n    --binning-vertical=<pixels>[,<mode>]";
    std::cerr << "\n                                 sets the number of adjacent vertical pixels to bin";
    std::cerr << "\n                                 <mode>: sum, average (default: sum)";
    std::cerr << "\n                                 ROI will refer to binned rows";
    std::cerr << "\n";
    std::cerr << "\ncamera options";
    std::cerr << "\n    --frame-rate=[<fps>]         set frame rate; limited by exposure";
    std::cerr << "\n    --exposure=[<µs>]            exposure time; \"auto\" to automatically set";
    std::cerr << "\n    --gain=[<num>]               gain; \"auto\" to automatically set;";
    std::cerr << "\n                                 for USB cameras units are dB";
    std::cerr << "\n";
    std::cerr << "\nacquisition options";
    std::cerr << "\n    --discard                    discard frames, if cannot keep up;";
    std::cerr << "\n                                 same as --buffer=1 (which is not a great setting)";
    std::cerr << "\n    --buffer=[<buffers>]         maximum buffer size before discarding frames";
    std::cerr << "\n                                 default: unlimited";
    std::cerr << "\n    --frame-trigger=[<type>]     'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-trigger=[<type>]      'line1', 'line2', 'line3', 'encoder'";
    std::cerr << "\n    --line-rate=[<num>]          line acquisition rate";
    std::cerr << "\n    --encoder-ticks=[<num>]      number of encoder ticks until the count resets";
    std::cerr << "\n                                 (reused for line number in frame in chunk mode)";
    std::cerr << "\n    --timeout=[<seconds>]        frame acquisition timeout; default " << default_timeout << "s";
    std::cerr << "\n";
    std::cerr << "\ntransport options";
    std::cerr << "\n    --packet-size=[<bytes>]      mtu size on camera side, should not be larger ";
    std::cerr << "\n                                 than your lan and network interface";
    std::cerr << "\n";
    std::cerr << "\nfilters";
    std::cerr << "\n    See \"cv-cat --help --verbose\" for a list of supported filters.";
    std::cerr << "\n";
    std::cerr << "\nBy default basler-cat will connect to the first device it finds. To choose a";
    std::cerr << "\nspecific camera use the --address or --serial-number options. For GigE cameras ";
    std::cerr << "\n<address> is the device ip address, for USB cameras it is the USB address.";
    std::cerr << "\nDetected cameras along with their addresses and serial numbers are shown by";
    std::cerr << "\n$ basler-cat --list-cameras --verbose.";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\n";
        std::cerr << snark::cv_mat::filters::usage();
        std::cerr << snark::cv_mat::serialization::options::type_usage();
    }
    std::cerr << "\nExample:";
    std::cerr << "\n    $ basler-cat \"resize=0.5;timestamp;view;null\"";
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
static snark::cv_mat::serialization::header cv_mat_header;
static bool is_packed; // quick and dirty
static comma::csv::options csv;
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

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
typedef std::pair< ChunkData, cv::Mat > chunk_pair_t;

template < typename T >
static void set( boost::posix_time::ptime& timestamp, boost::posix_time::ptime& t, const Pylon::GrabResult&, T& ) { timestamp = t; }

template < typename T >
static void set( ChunkData& d, boost::posix_time::ptime& t, const Pylon::GrabResult& result, T& camera )
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

static void output_result_status( const Pylon::GrabResult& result )
{
    std::cerr << "basler-cat: " << result.GetErrorDescription()
              << " (0x" << std::hex << result.GetErrorCode() << std::dec << ")" << std::endl;
    std::cerr << "            status: " << ( result.Status() == Pylon::Idle ? "idle" :
                                             result.Status() == Pylon::Queued ? "queued" :
                                             result.Status() == Pylon::Grabbed ? "grabbed" :
                                             result.Status() == Pylon::Canceled ? "canceled" :
                                             result.Status() == Pylon::Failed ? "failed" : "unknown" ) << std::endl;
}

template < typename T, typename P >
static P capture( T& camera, typename T::StreamGrabber_t& grabber )
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
    for( unsigned int i = 0; i < retries; ++i )
    {
        static comma::signal_flag is_shutdown;
        if( is_shutdown ) { return P(); }
        Pylon::GrabResult result;
        //camera.AcquisitionStart.Execute(); // acquire single image (since acquisition mode set so)
        if( !grabber.GetWaitObject().Wait( timeout ) ) // quick and dirty: arbitrary timeout
        {
            std::cerr << "basler-cat: timeout" << std::endl;
            grabber.CancelGrab();
            while( grabber.RetrieveResult( result ) ); // get all buffers back
            return P();
        }
        boost::posix_time::ptime current_time = boost::get_system_time();
        grabber.RetrieveResult( result );
        if( !result.Succeeded() )
        {
            std::cerr << "basler-cat: acquisition failed" << std::endl;
            output_result_status( result );
            std::cerr << "            run \"basler-cat --verbose\" and check your --packet-size settings" << std::endl;
            continue;
        }
        P pair;
        pair.second = cv::Mat( result.GetSizeY(), is_packed ? ( ( result.GetSizeX() * 3 ) / 2 ) : result.GetSizeX(), cv_mat_header.type ); // todo: seriously quick and dirty, does not scale to Mono10p; implement packed bytes layout properly 
        ::memcpy( pair.second.data, reinterpret_cast< const char* >( result.Buffer() ), pair.second.dataend - pair.second.datastart );
        // quick and dirty for now: rgb are not contiguous in basler camera frame
        if( cv_mat_header.type == CV_8UC3 || cv_mat_header.type == CV_16UC3 ) { cv::cvtColor( pair.second, pair.second, CV_RGB2BGR ); }
        set< T >( pair.first, current_time, result, camera );
        grabber.QueueBuffer( result.Handle(), NULL ); // requeue buffer
        return pair;
    }
    return P();
}

static void write( const chunk_pair_t& p )
{
    if( p.second.empty() || !std::cout.good() ) { return; }
    static comma::csv::binary_output_stream< Header > ostream( std::cout, csv );
    static Header header( cv_mat_header );
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

template < typename T > struct pixel_format;

template <> struct pixel_format< Pylon::CBaslerUsbCamera > // todo: support more formats
{
    typedef Basler_UsbCameraParams::PixelFormatEnums type_t;
    
    static type_t from_string( const std::string& name )
    {
        if( boost::algorithm::to_lower_copy( name ) == "mono8" ) { return Basler_UsbCameraParams::PixelFormat_Mono8; }
        if( boost::algorithm::to_lower_copy( name ) == "mono10" ) { return Basler_UsbCameraParams::PixelFormat_Mono10; }
        if( boost::algorithm::to_lower_copy( name ) == "mono10p" ) { return Basler_UsbCameraParams::PixelFormat_Mono10p; }
        if( boost::algorithm::to_lower_copy( name ) == "mono12" ) { return Basler_UsbCameraParams::PixelFormat_Mono12; }
        if( boost::algorithm::to_lower_copy( name ) == "mono12p" ) { return Basler_UsbCameraParams::PixelFormat_Mono12p; }
        COMMA_THROW( comma::exception, "pixel format \"" << name << "\" not implemented" );
    }
    
    static const char* to_string( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return "Mono8";
            case Basler_UsbCameraParams::PixelFormat_Mono10: return "Mono10";
            case Basler_UsbCameraParams::PixelFormat_Mono10p: return "Mono10p";
            case Basler_UsbCameraParams::PixelFormat_Mono12: return "Mono12";
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return "Mono12p";
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static unsigned int channels( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono10: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono10p: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono12: return 1;
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return 1;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static bool is_packed( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return false;
            case Basler_UsbCameraParams::PixelFormat_Mono10: return false;
            case Basler_UsbCameraParams::PixelFormat_Mono12: return false;
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return true;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static const char* to_image_type_string( type_t format )
    {
        switch( format )
        {
            case Basler_UsbCameraParams::PixelFormat_Mono8: return "ub";
            case Basler_UsbCameraParams::PixelFormat_Mono10: return "uw";
            //case Basler_UsbCameraParams::PixelFormat_Mono10p: return "uw";
            case Basler_UsbCameraParams::PixelFormat_Mono12: return "uw";
            case Basler_UsbCameraParams::PixelFormat_Mono12p: return "ub"; // quick and dirty
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
};

template <> struct pixel_format< Pylon::CBaslerGigECamera > // todo: support more formats
{
    typedef Basler_GigECameraParams::PixelFormatEnums type_t;
    
    static type_t from_string( const std::string& name )
    {
        if( boost::algorithm::to_lower_copy( name ) == "mono8" ) { return Basler_GigECameraParams::PixelFormat_Mono8; }
        if( boost::algorithm::to_lower_copy( name ) == "rgb8packed" ) { return Basler_GigECameraParams::PixelFormat_RGB8Packed; }
        COMMA_THROW( comma::exception, "pixel format \"" << name << "\" not implemented" );
    }
    
    static const char* to_string( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return "Mono8";
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return "RGB8Packed";
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static unsigned int channels( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return 1;
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return 3;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static bool is_packed( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return false;
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return false;
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
    
    static const char* to_image_type_string( type_t format )
    {
        switch( format )
        {
            case Basler_GigECameraParams::PixelFormat_Mono8: return "ub";
            case Basler_GigECameraParams::PixelFormat_RGB8Packed: return "3ub";
            default: COMMA_THROW( comma::exception, "pixel format " << format << " not implemented" );
        }
    }
};

template< typename T, typename P >
static void load_default_settings( T& camera, P user_set_selector_default )
{
    camera.UserSetSelector.SetValue( user_set_selector_default );
    camera.UserSetLoad.Execute();
}
static void load_default_settings( Pylon::CBaslerGigECamera& camera ) { load_default_settings( camera, Basler_GigECameraParams::UserSetSelector_Default ); }
static void load_default_settings( Pylon::CBaslerUsbCamera& camera ) { load_default_settings( camera, Basler_UsbCameraParams::UserSetSelector_Default ); }

template < typename T, typename P >
static void set_pixel_format( T& camera, P type )
{
    if( camera.PixelFormat() == type ) { return; }
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
        supported = supported || pixel_format< T >::to_string( type ) != e->GetSymbolic().c_str();
    }
    if( comma::verbose ) { std::cerr << std::endl; }
    if( !supported ) { COMMA_THROW( comma::exception, "pixel format \"" << pixel_format< T >::to_string( type ) << "\" is not supported by the camera" ); }
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
    comma::verbose << "pixel format set to " << pixel_format< T >::to_string( type ) << std::endl;
    return;
}

template < typename T >
static void set_pixel_format( T& camera, const comma::command_line_options& options )
{
    std::string pixel_format_string = options.value< std::string >( "--pixel-format", "" );
    typename ::pixel_format< T >::type_t pixel_format = pixel_format_string.empty() ? camera.PixelFormat() : ::pixel_format< T >::from_string( pixel_format_string );
    set_pixel_format( camera, pixel_format );
    cv_mat_options.type = ::pixel_format< T >::to_image_type_string( pixel_format );
    is_packed = ::pixel_format< T >::is_packed( pixel_format );
}

static std::string get_address( const Pylon::CDeviceInfo& device_info )
{
    std::string full_name = device_info.GetFullName().c_str();
    std::string device_class = device_info.GetDeviceClass().c_str();
    if( device_class == "BaslerGigE" )
    {
        // It would be nice to use Pylon::CBaslerGigEDeviceInfo::GetAddress()
        // but casting to that class appears not to work
        boost::regex address_regex( ".*#([0-9.]+):.*", boost::regex::extended );
        boost::smatch match;
        if( boost::regex_match( full_name, match, address_regex )) { return match[1]; }
    }
    else if( device_class == "BaslerUsb" ) { return full_name; }
    return "";
}

static void list_cameras()
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
            std::cerr << "\nAddress:    " << get_address( *it );
            std::cerr << "\nFull name:  " << it->GetFullName();
            std::cerr << std::endl;
        }
        else { std::cerr << it->GetFullName() << std::endl; }
    }
    if( comma::verbose && !devices.empty() ) { std::cerr << std::endl; }
}

static bool is_ip_address( std::string str )
{
    boost::regex ip_regex( "[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+" ); // somewhat naive
    return boost::regex_match( str, ip_regex );
}

static Pylon::IPylonDevice* create_device( const std::string& address, const std::string& serial_number )
{
    Pylon::CTlFactory& factory = Pylon::CTlFactory::GetInstance();
    if( !serial_number.empty() )
    {
        Pylon::CDeviceInfo device_info;
        device_info.SetSerialNumber( serial_number.c_str() );
        return factory.CreateDevice( device_info );
    }
    else if( !address.empty() )
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
    else
    {
        Pylon::DeviceInfoList_t devices;
        factory.EnumerateDevices( devices );
        std::cerr << "basler-cat: ";
        switch( devices.size() )
        {
            case 0:
                std::cerr << "no camera found" << std::endl;
                return NULL;
            case 1:
                std::cerr << "found 1 device: " << devices[0].GetFullName() << std::endl;
                break;
            default:
                std::cerr << "will connect to the first of " << devices.size() << " found devices:" << std::endl;
                Pylon::DeviceInfoList_t::const_iterator it;
                for( it = devices.begin(); it != devices.end(); ++it ) { std::cerr << "    " << it->GetFullName() << std::endl; }
        }
        return factory.CreateDevice( devices[0] );
    }
}

static bool chunk_mode = false;
static std::string filters;

static bool configure_trigger( Pylon::CBaslerUsbCamera& camera, const comma::command_line_options& options ) { return true; }

static bool configure_trigger( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
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

static void configure_chunk_mode( Pylon::CBaslerUsbCamera& camera ) { COMMA_THROW( comma::exception, "chunk mode not supported for USB cameras" ); }

static void configure_chunk_mode( Pylon::CBaslerGigECamera& camera )
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

static void set_frame_rate( Pylon::CBaslerGigECamera& camera, double frame_rate )
{
    camera.AcquisitionFrameRateEnable = true;
    camera.AcquisitionFrameRateAbs = frame_rate;
}

static void set_frame_rate( Pylon::CBaslerUsbCamera& camera, double frame_rate )
{
    camera.AcquisitionFrameRateEnable = true;
    camera.AcquisitionFrameRate = frame_rate;
}

static void set_exposure( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
{
    camera.ExposureMode = Basler_GigECameraParams::ExposureMode_Timed;
    if( options.exists( "--exposure" ))
    {
        std::string exposure = options.value< std::string >( "--exposure" );
        if ( exposure == "auto" )
        {
            camera.ExposureAuto = Basler_GigECameraParams::ExposureAuto_Once;
        }
        else
        {
            camera.ExposureAuto = Basler_GigECameraParams::ExposureAuto_Off;
            camera.ExposureTimeAbs = boost::lexical_cast< unsigned int >( exposure );
        }
    }
}

static void set_exposure( Pylon::CBaslerUsbCamera& camera, const comma::command_line_options& options )
{
    camera.ExposureMode = Basler_UsbCameraParams::ExposureMode_Timed;
    if( options.exists( "--exposure" ))
    {
        std::string exposure = options.value< std::string >( "--exposure" );
        if ( exposure == "auto" )
        {
            camera.ExposureAuto = Basler_UsbCameraParams::ExposureAuto_Once;
        }
        else
        {
            camera.ExposureAuto = Basler_UsbCameraParams::ExposureAuto_Off;
            camera.ExposureTime = boost::lexical_cast< unsigned int >( exposure );
        }
    }
}

static void set_gain( Pylon::CBaslerGigECamera& camera, const comma::command_line_options& options )
{
    camera.GainSelector = Basler_GigECameraParams::GainSelector_All;
    if( options.exists( "--gain" ))
    {
        std::string gain = options.value< std::string >( "--gain" );
        if ( gain == "auto" )
        {
            camera.GainAuto = Basler_GigECameraParams::GainAuto_Once;
        }
        else
        {
            camera.GainAuto = Basler_GigECameraParams::GainAuto_Off;
            camera.GainRaw = boost::lexical_cast< unsigned int >( gain );
        }
    }
}

static void set_gain( Pylon::CBaslerUsbCamera& camera, const comma::command_line_options& options )
{
    camera.GainSelector = Basler_UsbCameraParams::GainSelector_All;
    if( options.exists( "--gain" ))
    {
        std::string gain = options.value< std::string >( "--gain" );
        if ( gain == "auto" )
        {
            camera.GainAuto = Basler_UsbCameraParams::GainAuto_Once;
        }
        else
        {
            camera.GainAuto = Basler_UsbCameraParams::GainAuto_Off;
            camera.Gain = boost::lexical_cast< unsigned int >( gain );
        }
    }
}

static void set_line_rate( Pylon::CBaslerGigECamera& camera, unsigned int line_rate ) { camera.AcquisitionLineRateAbs = line_rate; }
static void set_line_rate( Pylon::CBaslerUsbCamera& camera, unsigned int ) { COMMA_THROW( comma::exception, "--line-rate not supported for USB cameras" ); }
static void set_packet_size( Pylon::CBaslerGigECamera& camera, unsigned int packet_size ) { camera.GevSCPSPacketSize = packet_size; }
static void set_packet_size( Pylon::CBaslerUsbCamera& camera, unsigned int ) { COMMA_THROW( comma::exception, "--packet-size not supported for USB cameras" ); }
static void set_socket_buffer_size( Pylon::CBaslerGigECamera::StreamGrabber_t& grabber, unsigned int socket_buffer_size ) { grabber.SocketBufferSize = socket_buffer_size; }
static void set_socket_buffer_size( Pylon::CBaslerUsbCamera::StreamGrabber_t&, unsigned int ) {}

static void set_test_image( Pylon::CBaslerGigECamera& camera, unsigned int test_image_num )
{
    switch( test_image_num )
    {
        case 0: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Off; break;
        case 1: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage1; break;
        case 2: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage2; break;
        case 3: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage3; break;
        case 4: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage4; break;
        case 5: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage5; break;
        case 6: camera.TestImageSelector = Basler_GigECameraParams::TestImageSelector_Testimage6; break;
        default: COMMA_THROW( comma::exception, "test image " << test_image_num << " is not supported. Choose a number from 1 to 6" );
    }
}

static void set_test_image( Pylon::CBaslerUsbCamera& camera, unsigned int test_image_num )
{
    switch( test_image_num )
    {
        case 0: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Off; break;
        case 1: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage1; break;
        case 2: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage2; break;
        case 3: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage3; break;
        case 4: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage4; break;
        case 5: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage5; break;
        case 6: camera.TestImageSelector = Basler_UsbCameraParams::TestImageSelector_Testimage6; break;
        default: COMMA_THROW( comma::exception, "test image " << test_image_num << " is not supported. Choose a number from 1 to 6" );
    }
}

template< typename T, typename P >
static void set_acquisition_mode( T& camera, P acquisition_mode )
{
    if( camera.AcquisitionMode() == acquisition_mode ) { return; }
    if( GenApi::IsWritable( camera.AcquisitionMode ) ) { camera.AcquisitionMode = acquisition_mode; }
    else { COMMA_THROW( comma::exception, "unable to set acquisition mode, parameter is not writable" ); }
}

static void set_continuous_acquisition_mode( Pylon::CBaslerGigECamera& camera ) { set_acquisition_mode( camera, Basler_GigECameraParams::AcquisitionMode_Continuous ); }
static void set_continuous_acquisition_mode( Pylon::CBaslerUsbCamera& camera ) { set_acquisition_mode( camera, Basler_UsbCameraParams::AcquisitionMode_Continuous ); }
static double get_exposure_time( const Pylon::CBaslerGigECamera& camera ) { return camera.ExposureTimeAbs(); }
static double get_gain( const Pylon::CBaslerGigECamera& camera ) { return camera.GainRaw(); }
static double get_frame_rate( const Pylon::CBaslerGigECamera& camera ) { return camera.ResultingFrameRateAbs(); }
static double get_exposure_time( const Pylon::CBaslerUsbCamera& camera ) { return camera.ExposureTime(); }
static double get_gain( const Pylon::CBaslerUsbCamera& camera ) { return camera.Gain(); }
static double get_frame_rate( const Pylon::CBaslerUsbCamera& camera ) { return camera.ResultingFrameRate(); }

template< typename T, typename P > static void set_binning_horizontal_mode( T& camera, P mode ) { camera.BinningHorizontalMode = mode; }
static void set_binning_horizontal_mode_sum( Pylon::CBaslerGigECamera& camera ) { set_binning_horizontal_mode( camera, Basler_GigECameraParams::BinningHorizontalMode_Sum ); }
static void set_binning_horizontal_mode_average( Pylon::CBaslerGigECamera& camera ) { set_binning_horizontal_mode( camera, Basler_GigECameraParams::BinningHorizontalMode_Average ); }
static void set_binning_horizontal_mode_sum( Pylon::CBaslerUsbCamera& camera ) { set_binning_horizontal_mode( camera, Basler_UsbCameraParams::BinningHorizontalMode_Sum ); }
static void set_binning_horizontal_mode_average( Pylon::CBaslerUsbCamera& camera ) { set_binning_horizontal_mode( camera, Basler_UsbCameraParams::BinningHorizontalMode_Average ); }

template< typename T, typename P > static void set_binning_vertical_mode( T& camera, P mode ) { camera.BinningVerticalMode = mode; }
static void set_binning_vertical_mode_sum( Pylon::CBaslerGigECamera& camera ) { set_binning_vertical_mode( camera, Basler_GigECameraParams::BinningVerticalMode_Sum ); }
static void set_binning_vertical_mode_average( Pylon::CBaslerGigECamera& camera ) { set_binning_vertical_mode( camera, Basler_GigECameraParams::BinningVerticalMode_Average ); }
static void set_binning_vertical_mode_sum( Pylon::CBaslerUsbCamera& camera ) { set_binning_vertical_mode( camera, Basler_UsbCameraParams::BinningVerticalMode_Sum ); }
static void set_binning_vertical_mode_average( Pylon::CBaslerUsbCamera& camera ) { set_binning_vertical_mode( camera, Basler_UsbCameraParams::BinningVerticalMode_Average ); }

template< typename T >
static void show_config( const T& camera, const comma::command_line_options& options )
{
    if( comma::verbose )
    {
        bool exposure_is_auto = ( options.value< std::string >( "--exposure", "" ) == "auto" );
        bool gain_is_auto = ( options.value< std::string >( "--gain", "" ) == "auto" );

        std::cerr << "basler-cat:     exposure: ";
        if( exposure_is_auto ) { std::cerr << "auto"; }
        else { std::cerr << get_exposure_time( camera ) << "µs"; }
        std::cerr << std::endl;

        std::cerr << "basler-cat:         gain: ";
        if( gain_is_auto ) { std::cerr << "auto"; }
        else { std::cerr << get_gain( camera ) << "dB"; }
        std::cerr << std::endl;

        // If frame rate is not explicitly set and exposure is set to auto
        // then we won't know the correct frame rate yet
        std::cerr << "basler-cat:   frame rate: ";
        if( !options.exists( "--frame-rate" ) && exposure_is_auto ) { std::cerr << "calculating..."; }
        else { std::cerr << get_frame_rate( camera ) << " fps"; }
        std::cerr << std::endl;

        std::cerr << "basler-cat: payload size: " << camera.PayloadSize() << " bytes" << std::endl;
        std::cerr << "basler-cat: pixel format: " << pixel_format< T >::to_string( camera.PixelFormat() ) << std::endl;
    }
}

static void show_transport_config( Pylon::CBaslerGigECamera& camera )
{
    comma::verbose << " packet size: " << camera.GevSCPSPacketSize() << " bytes" << std::endl;
}

static void show_transport_config( Pylon::CBaslerUsbCamera& camera )
{
}

static void show_config( Pylon::CBaslerGigECamera::StreamGrabber_t& grabber )
{
    comma::verbose << "socket buffer size: " << grabber.SocketBufferSize() << " bytes" << std::endl;
    comma::verbose << "   max buffer size: " << grabber.MaxBufferSize() << " bytes" << std::endl;
}

static void show_config( Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber ) { comma::verbose << "max buffer size: " << grabber.MaxBufferSize() << " bytes" << std::endl; }

static bool run_chunk_pipeline( Pylon::CBaslerGigECamera& camera, Pylon::CBaslerGigECamera::StreamGrabber_t& grabber, unsigned int max_queue_size, unsigned int max_queue_capacity )
{
    snark::tbb::bursty_reader< chunk_pair_t > reader( boost::bind( &capture< Pylon::CBaslerGigECamera, chunk_pair_t >, boost::ref( camera ), boost::ref( grabber ) ), max_queue_size, max_queue_capacity );
    tbb::filter_t< chunk_pair_t, void > writer( tbb::filter::serial_in_order, boost::bind( &write, _1 ));
    snark::tbb::bursty_pipeline< chunk_pair_t > pipeline;
    camera.AcquisitionStart.Execute();
    comma::verbose << "running in chunk mode..." << std::endl;
    pipeline.run( reader, writer );
    comma::verbose << "shutting down..." << std::endl;
    camera.AcquisitionStop();
    comma::verbose << "acquisition stopped" << std::endl;
    reader.join();
    camera.DestroyChunkParser( parser );
    return true;
}

static bool run_chunk_pipeline( Pylon::CBaslerUsbCamera& camera, Pylon::CBaslerUsbCamera::StreamGrabber_t& grabber, unsigned int max_queue_size, unsigned int max_queue_capacity ) { COMMA_THROW( comma::exception, "chunk mode not supported for USB cameras" ); }

template< typename C, typename G >
static bool run_simple_pipeline( C& camera, G& grabber, unsigned int max_queue_size, unsigned int max_queue_capacity )
{
    bool error = false;
    snark::cv_mat::serialization serialization( cv_mat_options );
    snark::tbb::bursty_reader< pair_t > reader( boost::bind( &capture< C, pair_t >, boost::ref( camera ), boost::ref( grabber ) ), max_queue_size, max_queue_capacity );
    snark::imaging::applications::pipeline pipeline( serialization, filters, reader );
    camera.AcquisitionStart.Execute();
    comma::verbose << "running..." << std::endl;
    pipeline.run();
    if( !pipeline.error().empty() ) { std::cerr << "basler-cat: \"" << pipeline.error() << "\"" << std::endl; error = true; }
    comma::verbose << "shutting down..." << std::endl;
    camera.AcquisitionStop();
    comma::verbose << "acquisition stopped" << std::endl;
    reader.join();
    return !error;
}

template< typename C, typename G >
static bool run_pipeline( C& camera, G& grabber, bool chunk_mode, unsigned int max_queue_size, unsigned int max_queue_capacity )
{
    if( chunk_mode ) { return run_chunk_pipeline( camera, grabber, max_queue_size, max_queue_capacity ); }
    else { return run_simple_pipeline( camera, grabber, max_queue_size, max_queue_capacity ); }
}

template< typename T >
static int run( T& camera, const comma::command_line_options& options )
{
    typedef T camera_t;
    cv_mat_options.header_only = options.exists( "--header-only" );
    cv_mat_options.no_header = options.exists( "--no-header" );
    timeout = options.value< double >( "--timeout", default_timeout ) * 1000.0;
    comma::verbose << "initialized camera" << std::endl;
    comma::verbose << "opening camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << "..." << std::endl;
    camera.Open();
    comma::verbose << "opened camera " << camera.GetDevice()->GetDeviceInfo().GetFullName() << std::endl;    
    load_default_settings( camera );
    set_pixel_format( camera, options );
    cv_mat_header = cv_mat_options.get_header();
    typename camera_t::StreamGrabber_t grabber( camera.GetStreamGrabber( 0 ) );
    grabber.Open();
    camera.OffsetX = 0;                 // reset before we get the maximum width
    unsigned int max_width = camera.Width.GetMax();
    unsigned int offset_x = options.value< unsigned int >( "--offset-x", 0 );
    if( offset_x >= max_width ) { std::cerr << "basler-cat: expected --offset-x less than " << max_width << ", got " << offset_x << std::endl; return 1; }
    unsigned int width = options.value< unsigned int >( "--width", max_width );
    if(( width + offset_x ) > max_width ) { width = max_width - offset_x; }
    camera.Width = width;
    camera.OffsetX = offset_x;          // but set _after_ we set the actual width
    camera.OffsetY = 0;                 // reset before we get the maximum height
    unsigned int max_height = camera.Height.GetMax();
    unsigned int offset_y = options.value< unsigned int >( "--offset-y", 0 );
    if( offset_y >= max_height ) { std::cerr << "basler-cat: expected --offset-y less than " << max_height << ", got " << offset_y << std::endl; return 1; }
    unsigned int height = options.value< unsigned int >( "--height", max_height );

    // todo: is the colour line 2098 * 3 or ( 2098 / 3 ) * 3 ?
    //unsigned int channels = num_channels( camera, cv_mat_header.type );
    //offset_y *= channels;
    //height *= channels;

    if(( height + offset_y ) > max_height ) { height = max_height - offset_y; }
    camera.Height = height;
    camera.OffsetY = offset_y;          // but set _after_ we set the actual height
    comma::verbose << "set width,height to " << width << "," << height << std::endl;

    if( options.exists( "--packet-size" )) { set_packet_size( camera, options.value< unsigned int >( "--packet-size" )); }

    if( options.exists( "--binning-horizontal" ) )
    {
        std::string s = options.value< std::string >( "--binning-horizontal" );
        std::vector< std::string > v = comma::split( s, ',' );
        camera.BinningHorizontal = boost::lexical_cast< unsigned int >( v[0] );
        for( unsigned int i = 1 ; i < v.size(); ++i )
        {
            if( v[i] == "sum" ) { set_binning_horizontal_mode_sum( camera ); }
            else if( v[i] == "average" ) { set_binning_horizontal_mode_average( camera ); }
            else { std::cerr << "basler-cat: invalid mode for --binning-horizontal: " << v[i] << std::endl; return 1; }
        }
    }
    else
    {
        camera.BinningHorizontal = 1;
    }
    if( options.exists( "--binning-vertical" ) )
    {
        std::string s = options.value< std::string >( "--binning-vertical" );
        std::vector< std::string > v = comma::split( s, ',' );
        camera.BinningVertical = boost::lexical_cast< unsigned int >( v[0] );
        for( unsigned int i = 1 ; i < v.size(); ++i )
        {
            if( v[i] == "sum" ) { set_binning_vertical_mode_sum( camera ); }
            else if( v[i] == "average" ) { set_binning_vertical_mode_average( camera ); }
            else { std::cerr << "basler-cat: invalid mode for --binning-vertical: " << v[i] << std::endl; return 1; }
        }
    }
    else
    {
        camera.BinningVertical = 1;
    }

    if( options.exists( "--reverse-x" ) ) { camera.ReverseX = true; }
    if( options.exists( "--reverse-y" ) ) { camera.ReverseY = true; }
    
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
    if( options.exists( "--frame-rate" )) { set_frame_rate( camera, options.value< double >( "--frame-rate" )); }
    else { camera.AcquisitionFrameRateEnable = false; }
    set_exposure( camera, options );
    set_gain( camera, options );
    if( options.exists( "--line-rate" )) { set_line_rate( camera, options.value< unsigned int >( "--line-rate" )); }
    if( GenApi::IsAvailable( camera.TestImageSelector ) ) { set_test_image( camera, options.value< unsigned int >( "--test-image", 0 )); }
    else { if( options.exists( "--test-image" )) { COMMA_THROW( comma::exception, "test image is not supported by this camera" ); } }
    show_config( camera, options );
    show_transport_config( camera );
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
    int return_value = 0;
    set_continuous_acquisition_mode( camera );
    try { if( !run_pipeline( camera, grabber, chunk_mode, max_queue_size, max_queue_size * 3 ) ) { return_value = 1; } }
    catch( std::exception& ex ) { std::cerr << "basler-cat: " << ex.what() << std::endl; return_value = 1; }
    grabber.FinishGrab();
    Pylon::GrabResult result;
    while( grabber.RetrieveResult( result ) ); // get all buffers back
    for( std::size_t i = 0; i < buffers.size(); ++i ) { grabber.DeregisterBuffer( buffer_handles[i] ); }
    grabber.Close();
    camera.Close();
    return return_value;
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
        if( options.exists( "--list-cameras" ) ) { list_cameras(); return 0; }
        comma::verbose << "PYLON_ROOT=" << ::getenv( "PYLON_ROOT" ) << std::endl;
        comma::verbose << "GENICAM_ROOT_V2_1=" << ::getenv( "GENICAM_ROOT_V2_1" ) << std::endl;
        comma::verbose << "initializing camera..." << std::endl;
        std::string address = options.value< std::string >( "--address", "" );
        std::string serial_number = options.value< std::string >( "--serial-number", "" );
        Pylon::IPylonDevice* device = create_device( address, serial_number );
        if( !device )
        {
            std::cerr << "basler-cat: unable to open camera";
            if( !address.empty() ) { std::cerr << " for address " << address; }
            std::cerr << std::endl;
            return 1;
        }
        filters = comma::join( options.unnamed( "--help,-h,--verbose,-v,--discard,--list-cameras,--header-only,--no-header", "-.*" ), ';' );
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
            encoder_ticks = options.value< unsigned int >( "--encoder-ticks" );
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
        Pylon::String_t device_class = device->GetDeviceInfo().GetDeviceClass();
        if ( device_class == "BaslerGigE" )
        {
            Pylon::CBaslerGigECamera camera;
            camera.Attach( device );
            return run( camera, options );
        }
        if( device_class == "BaslerUsb" )
        {
            Pylon::CBaslerUsbCamera camera;
            camera.Attach( device );
            return run( camera, options );
        }
        std::cerr << "basler-cat: unsupported device type of " << device_class << std::endl;
        return 1;
    }
#if ( PYLON_VERSION_MAJOR >= 5 )
    catch(const Pylon::GenericException& e) { std::cerr << "basler-cat: pylon exception: " << e.what() << std::endl; }
#endif
    catch( std::exception& ex ) { std::cerr << "basler-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "basler-cat: unknown exception" << std::endl; }
    return 1;
}
