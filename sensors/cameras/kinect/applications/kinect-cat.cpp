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

/// @authors john gardenier, vsevolod vlaskine

#include <Eigen/Core>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/string/string.h>
#include "../../../../visiting/traits.h"
#include "../../../../timing/timestamped.h"
#include "../../../../timing/traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "acquire data from kinect camera, output to stdout as ascii (default) or binary" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: kinect-cat [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --as-binary: output binary in default format; a convenience option" << std::endl;
    std::cerr << "    --output-fields; print default output fields to stdout and exit" << std::endl;
    std::cerr << "    --output-format; print default output format to stdout and exit" << std::endl;
    std::cerr << "    --version=[<version>]; kinect version; default: 2" << std::endl;
    std::cerr << "    --verbose; more info output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    else { std::cerr << "    run kinect-cat --help --verbose for details..." << std::endl; }
    std::cerr << std::endl;
}

namespace snark { namespace kinect {
    
struct color // quick and dirty
{
    unsigned char r, g, b;
    color( unsigned char r, unsigned char g, unsigned char b ) : r( r ), g( g ), b( b ) {}
    color() : r( 0 ), g( 0 ), b( 0 ) {}
};
    
struct point
{
    boost::posix_time::ptime t;
    Eigen::Vector3d coordinates;
    kinect::color color;
    Eigen::Vector2i index; // pixel index
    unsigned int block;
    point(): coordinates( Eigen::Vector3d::Zero() ), index( Eigen::Vector2i::Zero() ), block( 0 ) {}
};

struct frame // todo
{
    class const_iterator
    {
        public:
            const_iterator& operator++() { ++index_; return *this; }
            bool operator==( const const_iterator& rhs ) const { return index_ == rhs.index_; }
            bool operator!=( const const_iterator& rhs ) const { return !operator==( rhs ); }
            kinect::point operator*() const // todo: fill from frame
            {
                kinect::point p;
                p.t = frame_->t;
                p.block = frame_->block;
                p.coordinates = Eigen::Vector3d( 1, 2, 3 );
                p.color = kinect::color( 4, 5, 6 );
                p.index = Eigen::Vector2i( 7, 8 );
                return p;
            }
            
        private:
            friend class frame;
            unsigned int index_;
            const frame* frame_;
    };
    
    const_iterator begin() const { const_iterator it; it.index_ = 0; it.frame_ = this; return it; } // todo
    
    const_iterator end() const { const_iterator it; it.index_ = 4; it.frame_ = this; return it; } // todo
    
    frame() : block( 0 ) {}
    
    boost::posix_time::ptime t;
    unsigned int block;
};

class camera
{
    public:
        bool read() // todo: currently just a demo; read one kinect frame, whatever that means; put in current_
        {
            current_.t = boost::posix_time::microsec_clock::universal_time(); // it's better to get timestamp from the camera, if available
            return current_.block++ < 3;
        }
        
        const kinect::frame& frame() const { return current_; }
        
    private:
        kinect::frame current_;
};

} } // namespace snark { namespace kinect {

namespace comma { namespace visiting {

template <> struct traits< snark::kinect::color >
{
    template< typename K, typename V > static void visit( const K& k, const snark::kinect::color& p, V& v )
    {
        v.apply( "r", p.r );
        v.apply( "g", p.g );
        v.apply( "b", p.b );
    }
};

template <> struct traits< snark::kinect::point >
{
    template< typename K, typename V > static void visit( const K& k, const snark::kinect::point& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "coordinates", p.coordinates );
        v.apply( "color", p.color );
        v.apply( "index", p.index );
        v.apply( "block", p.block );
    }
};
    
} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        std::string version = options.value< std::string >( "--version", "2" );
        if( version != "2" ) { std::cerr << "kinect-cat: only version 2 is supported for now; got: \"" << version << "\"" << std::endl; return 1; }
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::kinect::point >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< snark::kinect::point >() << std::endl; return 0; }
        comma::csv::options csv( options );
        csv.full_xpath = true;
        if( !csv.binary() && options.exists( "--as-binary" ) ) { csv.format( comma::csv::format::value< snark::kinect::point >() ); }
        comma::csv::output_stream< snark::kinect::point > ostream( std::cout, csv );
        snark::kinect::camera camera;
        while( camera.read() && std::cout.good() ) // todo? if too slow, parallelize reading frame and output with tbb
        {
            for( snark::kinect::frame::const_iterator it = camera.frame().begin(); it != camera.frame().end(); ++it ) { ostream.write( *it ); }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "kinect-cat: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "kinect-cat: unknown exception" << std::endl; }
    return 1;
}
