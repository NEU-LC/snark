// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2018 Vsevolod Vlaskine
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

#include <algorithm>
#include <deque>
#include <memory>
#include <numeric>
#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include "../../imaging/cv_mat/filters.h"
#include "../../imaging/cv_mat/serialization.h"

static bool verbose = false;

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "read filters on stdin, apply to image, output image to stdout, keep image in memory meanwhile" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --undo-depth,-u=<depth>; number of undo steps; default: 10" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;

namespace snark { namespace image_edit {
        
class reader
{
    public:
        reader( const std::vector< std::string >& sources ): sources_( sources ), index_( 0 )
        {
            if( sources.empty() ) { std::cerr << "image-edit: please specify image or image source" << std::endl; exit( 1 ); }
        }
        
        pair_t read()
        {
            if( index_ >= sources_.size() ) { return pair_t(); }
            pair_t p;
            p.second = cv::imread( sources_[index_], cv::IMREAD_UNCHANGED ); // stream and video support: todo
            ++index_;
            return p;
        }
        
    private:
        std::vector< std::string > sources_;
        unsigned int index_;
};

} } // namespace snark { namespace image_edit {

// todo:
// - --help
// - apply: better semantics
// - input
//   - streams
//   - video
// - snark/pages
//   - set up
//   - blog
//   - publish

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        std::vector< std::string > history;
        snark::image_edit::reader reader( options.unnamed( "-h,--help,-v,--verbose", "-.*" ) );
        snark::cv_mat::serialization serialization;
        comma::signal_flag is_shutdown;
        std::deque< cv::Mat > image_edit_history;
        unsigned int undo_depth = options.value< unsigned int >( "--undo-depth,-h", 10 );
        for( unsigned int i = 0; !is_shutdown && std::cin.good(); ++i )
        {
            pair_t current = reader.read();
            if( current.second.empty() ) { break; }
            serialization.write_to_stdout( current );
            image_edit_history.clear();
            image_edit_history.push_back( cv::Mat() );
            current.second.copyTo( image_edit_history.back() );
            while( std::cin.good() )
            {
                std::string filter_string;
                std::getline( std::cin, filter_string );
                filter_string = comma::strip( filter_string );
                if( filter_string.empty() || filter_string[0] == '#' ) { continue; }
                std::cerr << i << ";" << filter_string << std::endl;
                const std::vector< std::string >& v = comma::split( filter_string, '=' );
                if( v[0] == "next" ) { break; }
                if( v[0] == "clear" ) { history.clear(); continue; }
                if( v[0] == "undo" )
                {
                    unsigned int depth = v.size() == 1 ? 1 : boost::lexical_cast< unsigned int >( v[1] );
                    depth = depth < ( image_edit_history.size() - 1 ) ? depth : ( image_edit_history.size() - 1 );
                    for( unsigned int k = 0; k < depth; ++k ) { image_edit_history.pop_back(); }
                    serialization.write_to_stdout( pair_t( current.first, image_edit_history.back() ) );
                    continue;
                }
                if( v[0] == "apply" ) { filter_string = comma::join( history, ';' ); } // quick and dirty
                history.push_back( filter_string );
                const std::vector< snark::cv_mat::filter >& filters = snark::cv_mat::filters::make( filter_string );
                pair_t filtered;
                filtered.first = current.first;
                image_edit_history.back().copyTo( filtered.second );
                for( auto& filter: filters ) { filtered = filter( filtered ); }
                serialization.write_to_stdout( filtered );
                image_edit_history.push_back( cv::Mat() );
                filtered.second.copyTo( image_edit_history.back() );
                if( image_edit_history.size() > undo_depth ) { image_edit_history.pop_front(); }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "cv-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "cv-calc: unknown exception" << std::endl; }
    return 1;
}
