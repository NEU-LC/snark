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

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/pipeline.h>
#include <tbb/task_scheduler_init.h>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include "../../../../imaging/cv_mat/filters.h"
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../tbb/queue.h"
#include "../flycapture.h"
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options.hpp>

static comma::signal_flag is_shutdown;
static bool verbose;
static unsigned int emptyFrameCounter = 0;
static unsigned int discard_more_than;
typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;
static snark::tbb::queue< Pair > queue;
static boost::scoped_ptr< snark::camera::flycapture::multicam > multicam;
static bool running = true;

// static void spin_()
// {
//     static unsigned int i = 0;
//     static char spinner[] = { '-', '/', '|', '\\' };
//     if( i % 3 == 0 ) { std::cerr << '\r' << spinner[ i / 3 ] << "   "; }
//     if( ++i >= 12 ) { i = 0; }
// }

// static void on_frame_( const Pair& p ) // quick and dirty
// {
//     std::cerr << "on_frame_" << std::endl;
//     if( p.second.size().width == 0 )
//     {
//         emptyFrameCounter++;
//         if( emptyFrameCounter > 20 )
//         {
//             COMMA_THROW( comma::exception, "got lots of empty frames, check that the packet size in the camera matches the mtu on your machine" );
//         }
//         if( verbose )
//         {
//             std::cerr << "flycapture-multicam: got empty frame" << std::endl;
//         }
//         return;
//     }
//     emptyFrameCounter = 0;
//     Pair q;
//     if( is_shutdown || !running ) { return queue.push( q ); } // to force read exit
//     q.first = p.first;
//     p.second.copyTo( q.second ); // quick and dirty: full copy; todo: implement circular queue in flycapture::multicam?
//     queue.push( q );
    
//     if( verbose ) { spin_(); }
//     if( discard_more_than > 0 )
//     {
//         int size = queue.size();
//         if( size > 1 )
//         {
//             int size_to_discard = size - discard_more_than;
//             Pair p;
//             for( int i = 0; i < size_to_discard; ++i ) { queue.pop( p ); } // clear() is not thread-safe
//             if( verbose && size_to_discard > 0 ) { std::cerr << "flycapture-multicam: discarded " << size_to_discard << " frames" << std::endl; }
//         }
//     }    
// }

// static Pair read_( tbb::flow_control& flow )
// {
//     if( queue.empty() || is_shutdown || !multicam->good() || !std::cout.good() || !running )
//     {
//         flow.stop();
//         return Pair();
//     }
//     Pair p;
//     queue.pop( p );
//     return p;
// }

// static void write_( snark::cv_mat::serialization& serialization, Pair p )
// {
//     if( p.second.size().width == 0 )
//     {
//         running = false;
//     }
//     static std::vector< char > buffer;
//     buffer.resize( serialization.size( p ) );
//     serialization.write( std::cout, p );
// }

int main( int argc, char** argv )
{
    try
    {
        std::vector<std::string> camera_strings;
        std::string fields;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        std::vector<snark::camera::flycapture::multicam::camera_pair> cameras;

        description.add_options()
            ( "help,h", "display help message" )
            ( "camera", boost::program_options::value< std::vector<std::string> >(), "a camera_string specifying the serial to connect to as well as the attributes and filters" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            // ( "list-attributes", "output current camera attributes" )
            ( "list-cameras", "list all cameras and exit" )
            ( "verbose,v", "be more verbose" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" );

        boost::program_options::variables_map vm;
        boost::program_options::positional_options_description p_opts;
        p_opts.add("camera", -1);
        boost::program_options::store( 
            boost::program_options::command_line_parser(argc, argv).options(description).positional(p_opts).run(),
            vm
        );
        boost::program_options::notify( vm );
        // boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();

        verbose = vm.count( "verbose" );
        if ( vm.count( "help" ) )
        {
            std::cerr << "acquire images from a point grey flycapture camera, same as flycapture-multicam but using a multicam " << std::endl;
            std::cerr << "instead of a thread to acquire the images" << std::endl;
            std::cerr << "output to stdout as serialized cv::Mat" << std::endl;
            std::cerr << "usage: flycapture-capture [<options>] camera_string[,camera_string,...]" << std::endl;
            std::cerr << "  camera_string: serial[,set_pairs[,filters]]" << std::endl;
            std::cerr << "known bug: freezes on slow consumers even with --discard, use flycapture-multicam instead" << std::endl;
            std::cerr << description << std::endl;
            if (verbose) { std::cerr << snark::cv_mat::filters::usage() << std::endl; }
            return 1;
        }

        if( vm.count( "list-cameras" ) )
        {
            const std::vector< unsigned int >& list = snark::camera::flycapture::list_camera_serials();
            for( std::size_t i = 0; i < list.size(); ++i ) // todo: serialize properly with name-value
            {
                std::cout << snark::camera::flycapture::describe_camera(list[i]) << std::endl;
            }
            return 0;
        }

        if( vm.count( "header" ) + vm.count( "no-header" ) > 1 )
        { COMMA_THROW( comma::exception, "--header, and --no-header are mutually exclusive" ); }
        
        if (vm.count("camera") < 1)
        { COMMA_THROW(comma::exception, "specify at least one camera serial"); }
        camera_strings = vm["camera"].as< std::vector<std::string> >();
        for (auto camera_string : camera_strings)
        {
            snark::camera::flycapture::attributes_type attributes;
            auto fields = comma::split( camera_string, "," );
            if (fields.size() < 1) { COMMA_THROW(comma::exception, "expected camera serial, got none"); }
            uint serial = boost::lexical_cast<uint>(fields[0]);
            if (fields.size() >= 2) {
                std::cerr << "set " << fields[1] << std::endl;
                comma::name_value::map set_map( fields[1], ';', '=' );
                attributes.insert( set_map.get().begin(), set_map.get().end() );
            }
            // if (fields.size() >= 3) { //TODO: implement per-camera filters
            //     auto& filterStrings = fields[2];
            // }
            cameras.push_back( std::make_pair(serial, attributes) );
        }
        
        if ( vm.count( "discard" ) ) { discard = 1; }
        discard_more_than = discard;

        comma::csv::format format;
        for (auto v : comma::split( fields, "," ))
        {
            if( v == "t" ) { format += "t"; }
            else { format += "ui"; }
        }

        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) )
        { serialization.reset( new snark::cv_mat::serialization( "", format ) ); }
        else
        { serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) ); }       
        
        if( verbose ) { std::cerr << "flycapture-multicam: connecting..." << std::endl; }
        // static snark::camera::flycapture::multicam multicam(cameras);
        multicam.reset( new snark::camera::flycapture::multicam( cameras ) );
        if( verbose ) { std::cerr << "flycapture-multicam: connected" << std::endl; }

        if( verbose ) { std::cerr << "flycapture-multicam: starting loop" << std::endl; }
        while( !is_shutdown && running )
        {
            // queue.wait();
            for (int i = 0; i < 20 ; ++i) 
            {
                boost::this_thread::sleep( boost::posix_time::milliseconds( 100 ) );
            }
            break;
        }
        if( verbose ) { std::cerr << "flycapture-multicam: exited loop" << std::endl; }
        
        if( is_shutdown && verbose ) { std::cerr << "flycapture-multicam: caught signal" << std::endl; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "flycapture-multicam: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "flycapture-multicam: unknown exception" << std::endl;
    }
    return 1;
}

