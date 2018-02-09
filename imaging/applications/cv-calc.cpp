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

#include <algorithm>
#include <memory>
#include <numeric>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/math/interval.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../imaging/cv_mat/filters.h"
#include "../../imaging/cv_mat/serialization.h"
#include "../../imaging/cv_mat/detail/life.h"

const char* name = "cv-calc: ";

static void usage( bool verbose=false )
{
    std::cerr << std::endl;
    std::cerr << "performs verious image manipulation or calculations on cv image streams" << std::endl;
    std::cerr << std::endl;
    std::cerr << "essential difference between cv-calc and  cv-cat" << std::endl;
    std::cerr << "    cv-cat takes one image in, applies operations, outputs one image out; e.g. cv-cat cannot skip images" << std::endl;
    std::cerr << "    cv-calc, depending on operation, may output multiple images per an one input image, skip images, or have just numeric outputs, etc" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat images.bin | cv-calc <operation> [<options>] > processed.bin " << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    draw: draw on the image primitives defined in the image header; skip a primitive if its dimensions are zero" << std::endl;
    std::cerr << "    format: output header and data format string in ascii" << std::endl;
    std::cerr << "    grep: output only images that satisfy conditions" << std::endl;
    std::cerr << "    header: output header information in ascii csv" << std::endl;
    std::cerr << "    life: take image on stdin, output game of life on each channel" << std::endl;
    std::cerr << "    mean: output image means for all image channels appended to image header" << std::endl;
    std::cerr << "    roi: given cv image data associated with a region of interest, either set everything outside the region of interest to zero or crop it" << std::endl;
    std::cerr << "    stride: stride through the image, output images of kernel size for each pixel" << std::endl;
    std::cerr << "    thin: thin image stream by discarding some images" << std::endl;
    std::cerr << "    unstride-positions: take stride index and positions within the stride, append positions in original (unstrided) image" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --binary=[<format>]: binary format of header; default: operation dependent, see --header-format" << std::endl;
    std::cerr << "    --fields=<fields>; fields in header; default: operation dependent, see --header-fields" << std::endl;
    std::cerr << "    --flush; flush after every image" << std::endl;
    std::cerr << "    --input=<options>; default values for image header; e.g. --input=\"rows=1000;cols=500;type=ub\", see serialization options" << std::endl;
    std::cerr << "    --header-fields; show header fields and exit" << std::endl;
    std::cerr << "    --header-format; show header format and exit" << std::endl;
    std::cerr << "    --output-fields; show output fields and exit" << std::endl;
    std::cerr << "    --output-format; show output format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "serialization options" << std::endl;
    if( verbose ) { std::cerr << snark::cv_mat::serialization::options::usage() << std::endl; } 
    else { std::cerr << "    run --help --verbose for more details..." << std::endl; }
    std::cerr << std::endl;
    std::cerr << "operation options" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    draw" << std::endl;
    std::cerr << "        --circles=<options>: draw circles given in the image header; fields: x,y,radius" << std::endl;
    std::cerr << "        --labels=<options>: draw labels given in the image header; fields: x,y,text" << std::endl;
    std::cerr << "        --rectangles=<options>: rectangles as min and max given in the image header; fields: min/x,min/y,max/x,max/y" << std::endl;
    std::cerr << "            <options>:<size>[,color=<r>,<g>,<b>][,weight=<weight>]" << std::endl;
    std::cerr << "                <size>: number of primitives" << std::endl;
    std::cerr << "                <r>,<g>,<b>: line colour as unsigned-byte rgb; default: 0,0,0 (black)" << std::endl;
    std::cerr << "                <weight>: line weight; default: 1" << std::endl;
    std::cerr << "        fields: t,rows,cols,type,circles,labels,rectangles" << std::endl;
    std::cerr << "        example: todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    grep" << std::endl;
    std::cerr << "        --filter,--filters=[<filters>]; apply --non-zero logic to the image with filters applied, not to image itself" << std::endl;
    std::cerr << "                                        run cv-cat --help --verbose for filters available" << std::endl;
    std::cerr << "        --non-zero=[<what>]; output only images that have non-zero pixels" << std::endl;
    std::cerr << "            <what>" << std::endl;
    std::cerr << "                ratio,[<min>][,<max>]: output only images with number of non-zero pixels within the limits of given ratios, e.g:" << std::endl;
    std::cerr << "                                           --non-zero=ratio,0.2,0.8: output images that have from 20 to 80% of non-zero pixels" << std::endl;
    std::cerr << "                                           --non-zero=ratio,,0.8: output images that have up to 80% of non-zero pixels" << std::endl;
    std::cerr << "                                           --non-zero=ratio,0.8: output images that have at least 80% of non-zero pixels" << std::endl;
    std::cerr << "                size,[<min>][,<max>]: output only images with number of non-zero pixels within given limits" << std::endl;
    std::cerr << "                                      lower limit inclusive, upper limit exclusive; e.g" << std::endl;
    std::cerr << "                                          --non-zero=size,10,1000: output images that have from 10 to 999 non-zero pixels" << std::endl;
    std::cerr << "                                          --non-zero=size,10: output images that have at least 10 non-zero pixels" << std::endl;
    std::cerr << "                                          --non-zero=size,,1000: output images that have not more than 999 non-zero pixels" << std::endl;
    std::cerr << "                                          --non-zero=size,,1: output images with all pixels zero (makes sense only when used with --filters" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    life" << std::endl;
    std::cerr << "        --exit-on-stability: exit, if no change" << std::endl;
    std::cerr << "        --procreation-treshold,--procreation=[<threshold>]: todo: document; default: 3.0" << std::endl;
    std::cerr << "        --stability-treshold,--stability=[<threshold>]: todo: document; default: 4.0" << std::endl;
    std::cerr << "        --step=[<step>]: todo: document; default: 1.0" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    mean" << std::endl;
    std::cerr << "        --threshold=[<thresh>]: apply a mask (binary threshold) and only calculate mean on pixel matching the mask." << std::endl;
    std::cerr << "              default: calculate a mean on all pixels" << std::endl;
    std::cerr << "        default output fields: t,rows,cols,type,mean,count" << std::endl;
    std::cerr << "                               count: total number of non-zero pixels used in calculating the mean" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    roi" << std::endl;
    std::cerr << "        --crop: crop to roi and output instead of setting region outside of roi to zero" << std::endl;
    std::cerr << "        --no-discard; do not discards frames where the roi is not seen" << std::endl;
    std::cerr << "        --permissive,--show-partial; allow partial overlaps of roi and input image, default: if partial roi and image overlap, set entire image to zeros." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    stride" << std::endl;
    std::cerr << "        --filter,--filters=[<filters>]; see grep operation; added to stride for performance" << std::endl;
    std::cerr << "        --input=[<options>]; input options; run cv-cat --help --verbose for details" << std::endl;
    std::cerr << "        --non-zero=[<what>]; see grep operation; added to stride for performance" << std::endl;
    std::cerr << "        --output=[<options>]; output options; run cv-cat --help --verbose for details" << std::endl;
    std::cerr << "        --padding=[<padding>]; padding, 'same' or 'valid' (see e.g. tensorflow for the meaning); default: valid" << std::endl;
    std::cerr << "        --shape,--kernel,--size=<x>,<y>; image size" << std::endl;
    std::cerr << "        --strides=[<x>,<y>]; stride size; default: 1,1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    thin" << std::endl;
    std::cerr << "        by thinning rate" << std::endl;
    std::cerr << "            --deterministic; output frames at a given thinning rate with as uniform intervals as possible" << std::endl;
    std::cerr << "                             default: output frames at a given thinning rate at random with uniform distribution" << std::endl;
    std::cerr << "            --rate=<rate>; thinning rate between 0 and 1" << std::endl;
    std::cerr << "        by frames per second" << std::endl;
    std::cerr << "            --from-fps,--input-fps=<fps>; input fps (since it is impossible to know it upfront)" << std::endl;
    std::cerr << "            --to-fps,--fps=<fps>; thin to a given <fps>, same as --rate=<to-fps>/<from-fps> --deterministic" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    unstride-positions" << std::endl;
    std::cerr << "        --positions=<n>: number of positions in stride; fields: positions[i]/x,positions[i]/y, (default: 1)" << std::endl;
    std::cerr << "        --shape,--kernel,--size=<x>,<y>; (tile) image size (see: stride operation)" << std::endl;
    std::cerr << "        --strides=[<x>,<y>]; stride size; default: 1,1 (see: stride operation)" << std::endl;
    std::cerr << "        --unstrided-size,--unstrided=<width>,<height>; original (unstrided) image size" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    draw" << std::endl;
    std::cerr << "        cv-cat --file image.jpg \\" << std::endl;
    std::cerr << "            | csv-paste 'value=50,100,100,200,150,150,250,250,200,200,100,250,150,some_text;binary=13ui,s[64]' \"-;binary=t,3ui,s[$image_size_in_bytes]\" \\" << std::endl;
    std::cerr << "            | cv-calc draw --binary=13ui,s[64],t,3ui \\" << std::endl;
    std::cerr << "                           --fields rectangles,circles,labels,t,rows,cols,type  \\" << std::endl;
    std::cerr << "                           --rectangles=2,weight=3,color/r=255 \\" << std::endl;
    std::cerr << "                           --circles=1,weight=3,color/b=255 \\" << std::endl;
    std::cerr << "                           --labels=1,weight=2,color/r=255 \\" << std::endl;
    std::cerr << "            | tail -c<original image size + 20> \\" << std::endl;
    std::cerr << "            | cv-cat --output no-header 'encode=jpg' > output.jpg" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    header" << std::endl;
    std::cerr << "        cat data.bin | cv-calc header" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    format" << std::endl;
    std::cerr << "        cat data.bin | cv-calc format" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    life" << std::endl;
    std::cerr << "        a combination of parameters producing fairly long-living colony:" << std::endl;
    std::cerr << "        cv-cat --file ~/tmp/some-image.jpg | cv-calc life --procreation 3 --stability 5.255 --step 0.02 | cv-cat \"count;view;null\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    roi" << std::endl;
    std::cerr << "        Setting everything but the roi rectangle to 0 for all images" << std::endl;
    std::cerr << "        ROI fields must be pre-pended. This roi is is a square of (100,100) to (300,300)" << std::endl;
    std::cerr << "        Given a cv-cat image stream with format 't,3ui,s[1572864]'." << std::endl;
    std::cerr << std::endl;
    std::cerr << "        cat data.bin | csv-paste \"value=100,100,300,300;binary=4i\" \"-;binary=t,3ui,s[1572864]\" \\" << std::endl;
    std::cerr << "            | cv-calc roi -v | csv-bin-cut '4i,t,3ui,s[1572864]' --fields 5-9 >output.bin" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        Explicity specifying fields. Image payload data field is not specified for cv-calc, not set for --binary either" << std::endl;
    std::cerr << "        The user must explcitly list all four roi fields. Using 'min,max' is not possible." << std::endl;
    std::cerr << std::endl;
    std::cerr << "        cat data.bin | csv-paste \"value=100,100,999,300,300;binary=5i\" \"-;binary=t,3ui,s[1572864]\" \\" << std::endl;
    std::cerr << "            | cv-calc roi --fields min/x,min/y,,max/x,max/y,t,rows,cols,type --binary '5i,t,3ui' \\" << std::endl;
    std::cerr << "            | csv-bin-cut '5i,t,3ui,s[1572864]' --fields 6-10 >output.bin" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    unstride-positions" << std::endl;
    std::cerr << "        <process strides, output stride index and rectangle> |" << std::endl;
    std::cerr << "            cv-calc unstride-positions --fields index,positions --positions 2 --size 600,400 --strides 300,200 --unstrided-size 1200,800" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct positions_t
{
    struct config
    {
        comma::uint32 size;
        config() : size( 1 ) {}
    };
    
    std::vector< cv::Point2d > positions;

    positions_t() {}
    positions_t( const comma::command_line_options& options )
    {
        std::string config_string = options.value< std::string >( "--positions", "1" );
        if( config_string.empty() ) { return; }
        const auto& config = comma::name_value::parser( "size", ',', '=' ).get< positions_t::config >( config_string );
        positions.resize( config.size );
    }
};

struct stride_positions_t : public positions_t
{
    comma::uint32 index;
    stride_positions_t() {}
    stride_positions_t( const comma::command_line_options& options ) : positions_t( options ) {}
};

struct extents
{
    cv::Point2i min;
    cv::Point2i max;
    extents(): min( 0, 0 ), max( 0, 0 ) {}
};

namespace snark { namespace imaging { namespace operations { namespace draw {

class shapes
{
public:
    struct color
    {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        color(): r( 0 ), g( 0 ), b( 0 ) {}
        operator cv::Scalar() const { return cv::Scalar( b, g, r ); }
    };
    
    struct properties
    {
        shapes::color color;
        comma::uint32 weight;
        properties() : weight( 1 ) {}
    };
    
    struct config : public properties
    {
        comma::uint32 size;
        config() : size( 1 ) {}
    };
    
    //void draw( cv::Mat m ) const { cv::circle( m, center, radius, color, thickness, line_type, shift ); }
    
    struct rectangle : public ::extents
    {
        shapes::properties properties; // todo: record-wise support
        rectangle() {}
        rectangle( const shapes::properties& properties ): properties( properties ) {}
        bool draw( cv::Mat m ) const
        {
            if( max.x == 0 && min.x == 0 && max.y == 0 && max.y == 0 ) { return false; }
            cv::rectangle( m, min, max, properties.color, properties.weight ); // CV_AA );
            return true;
        }
    };
    
    struct circle
    {
        cv::Point2i centre;
        double radius;
        shapes::properties properties; // todo: record-wise support
        circle() : centre( 0, 0 ), radius( 0 ) {}
        circle( const shapes::properties& properties ): properties( properties ) {}
        bool draw( cv::Mat m ) const
        {
            if( !comma::math::less( 0, radius ) ) { return false; }
            cv::circle( m, centre, radius, properties.color, properties.weight, CV_AA );
            return true;
        }
    };
    
    struct label
    {
        cv::Point2i position;
        std::string text;
        shapes::properties properties; // todo: record-wise support
        label() : position( 0, 0 ) {}
        label( const shapes::properties& properties ): properties( properties ) {}
        bool draw( cv::Mat m ) const
        {
            if( text.empty() ) { return false; }
            cv::putText( m, text, position, cv::FONT_HERSHEY_SIMPLEX, 1.0, properties.color, properties.weight, CV_AA );
            return true;
        }
    };
    
    std::vector< shapes::circle > circles;
    std::vector< shapes::label > labels;
    std::vector< shapes::rectangle > rectangles;
    
    shapes() {}
    
    shapes( const comma::command_line_options& options )
    {
        init_( rectangles, options, "--rectangles" );
        init_( circles, options, "--circles" );
        init_( labels, options, "--labels" );
    }
    
    void draw( cv::Mat m ) const
    {
        draw_( m, circles );
        draw_( m, labels );
        draw_( m, rectangles );
    }
    
    
private:
    template < typename T > void init_( std::vector< T >& s, const comma::command_line_options& options, const std::string& what )
    {
        std::string config_string = options.value< std::string >( what, "" );
        if( config_string.empty() ) { return; }
        const auto& config = comma::name_value::parser( "size", ',', '=' ).get< snark::imaging::operations::draw::shapes::config >( config_string );
        s.resize( config.size, T( static_cast< const shapes::properties& >( config ) ) );
    }
    template < typename T > void draw_( cv::Mat m, const std::vector< T >& s ) const { for( unsigned int i = 0; i < s.size(); s[i].draw( m ), ++i ); }
};

} } } } // namespace snark { namespace imaging { namespace operations { namespace draw {

namespace comma { namespace visiting {

template <> struct traits< positions_t::config >
{
    template < typename Key, class Visitor > static void visit( const Key&, positions_t::config& p, Visitor& v )
    {
        v.apply( "size", p.size );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const positions_t::config& p, Visitor& v )
    {
        v.apply( "size", p.size );
    }
};

template <> struct traits< positions_t >
{
    template < typename Key, class Visitor > static void visit( const Key&, positions_t& p, Visitor& v )
    {
        v.apply( "positions", p.positions );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const positions_t& p, Visitor& v )
    {
        v.apply( "positions", p.positions );
    }
};

template <> struct traits< stride_positions_t >
{
    template < typename Key, class Visitor > static void visit( const Key& k, stride_positions_t& p, Visitor& v )
    {
        v.apply( "index", p.index );
        traits< ::positions_t >::visit( k, p, v );
    }

    template < typename Key, class Visitor > static void visit( const Key& k, const stride_positions_t& p, Visitor& v )
    {
        v.apply( "index", p.index );
        traits< ::positions_t >::visit( k, p, v );
    }
};

template < typename T > struct traits< cv::Point_< T > >
{
    template < typename Key, class Visitor > static void visit( const Key&, cv::Point_< T >& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
    }
    
    template < typename Key, class Visitor > static void visit( const Key&, const cv::Point_< T >& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
    }
};

template <> struct traits< ::extents >
{
    template < typename Key, class Visitor > static void visit( const Key&, ::extents& p, Visitor& v )
    {
        v.apply( "min", p.min );
        v.apply( "max", p.max );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const ::extents& p, Visitor& v )
    {
        v.apply( "min", p.min );
        v.apply( "max", p.max );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes::color >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::imaging::operations::draw::shapes::color& p, Visitor& v )
    {
        unsigned int r = p.r;
        v.apply( "r", r );
        p.r = r;
        unsigned int g = p.g;
        v.apply( "g", g );
        p.g = g;
        unsigned int b = p.b;
        v.apply( "b", b );
        p.b = b;
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::imaging::operations::draw::shapes::color& p, Visitor& v )
    {
        v.apply( "r", comma::uint32( p.r ) );
        v.apply( "g", comma::uint32( p.g ) );
        v.apply( "b", comma::uint32( p.b ) );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes::properties >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::imaging::operations::draw::shapes::properties& p, Visitor& v )
    {
        v.apply( "color", p.color );
        v.apply( "weight", p.weight );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::imaging::operations::draw::shapes::properties& p, Visitor& v )
    {
        v.apply( "color", p.color );
        v.apply( "weight", p.weight );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes::config >
{
    template < typename Key, class Visitor > static void visit( const Key& k, snark::imaging::operations::draw::shapes::config& p, Visitor& v )
    {
        v.apply( "size", p.size );
        traits< snark::imaging::operations::draw::shapes::properties >::visit( k, p, v );
    }

    template < typename Key, class Visitor > static void visit( const Key& k, const snark::imaging::operations::draw::shapes::config& p, Visitor& v )
    {
        v.apply( "size", p.size );
        traits< snark::imaging::operations::draw::shapes::properties >::visit( k, p, v );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes::rectangle >
{
    template < typename Key, class Visitor > static void visit( const Key& k, snark::imaging::operations::draw::shapes::rectangle& p, Visitor& v )
    {
        traits< ::extents >::visit( k, p, v );
    }

    template < typename Key, class Visitor > static void visit( const Key& k, const snark::imaging::operations::draw::shapes::rectangle& p, Visitor& v )
    {
        traits< ::extents >::visit( k, p, v );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes::circle >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::imaging::operations::draw::shapes::circle& p, Visitor& v )
    {
        v.apply( "centre", p.centre );
        v.apply( "radius", p.radius );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::imaging::operations::draw::shapes::circle& p, Visitor& v )
    {
        v.apply( "centre", p.centre );
        v.apply( "radius", p.radius );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes::label >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::imaging::operations::draw::shapes::label& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "text", p.text );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::imaging::operations::draw::shapes::label& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "text", p.text );
    }
};

template <> struct traits< snark::imaging::operations::draw::shapes >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::imaging::operations::draw::shapes& p, Visitor& v )
    {
        v.apply( "circles", p.circles );
        v.apply( "labels", p.labels );
        v.apply( "rectangles", p.rectangles );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::imaging::operations::draw::shapes& p, Visitor& v )
    {
        v.apply( "circles", p.circles );
        v.apply( "labels", p.labels );
        v.apply( "rectangles", p.rectangles );
    }
};

} } // namespace comma { namespace visiting {

static bool verbose = false;

namespace snark { namespace imaging { namespace operations {
    
namespace grep {
    
class non_zero
{ 
    public:
        non_zero() {}
        non_zero( const std::string s )
        {
            if( s.empty() ) { return; }
            const std::vector< std::string >& v = comma::split( s, ',' );
            if( v[0] == "ratio" )
            { 
                if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected --non-zero=ratio,<min>,<max>; got --non-zero=ratio" ); }
                if( !v[1].empty() ) { ratio_.first = boost::lexical_cast< double >( v[1] ); }
                if( v.size() > 2 && !v[2].empty() ) { ratio_.second = boost::lexical_cast< double >( v[2] ); }
                return;
            }
            if( v[0] == "size" )
            {
                if( v.size() < 2 ) { COMMA_THROW( comma::exception, "expected --non-zero=size,<min>,<max>; got --non-zero=size" ); }
                if( !v[1].empty() ) { size_.first = boost::lexical_cast< unsigned int >( v[1] ); }
                if( v.size() > 2 && !v[2].empty() ) { size_.second = boost::lexical_cast< unsigned int >( v[2] ); }
                return;
            }
            COMMA_THROW( comma::exception, "--non-zero: expected 'ratio' or 'size', got: '" << v[0] << "'" );
        }
        operator bool() const { return static_cast< bool >( ratio_.first ) || static_cast< bool >( ratio_.second ) || static_cast< bool >( size_.first ) || static_cast< bool >( size_.second ); }
        void size( unsigned int image_size )
        { 
            if( ratio_.first ) { size_.first = image_size * *ratio_.first; }
            if( ratio_.second ) { size_.second = image_size * *ratio_.second; }
        }
        bool keep( unsigned int count ) const { return ( !size_.first || *size_.first <= count ) && ( !size_.second || count < *size_.second ); }
        bool keep( const cv::Mat& m ) const { return keep( count( m ) ); }
        unsigned int count( const cv::Mat& m ) const
        {
            static std::vector< char > zero_pixel( m.elemSize(), 0 );
            std::vector< unsigned int > counts( m.rows, 0 );
            tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, m.rows )
                                , [&]( const tbb::blocked_range< std::size_t >& r )
                                {
                                    for( unsigned int i = r.begin(); i < r.end(); ++i )
                                    {
                                        for( const unsigned char* ptr = m.ptr( i ); ptr < m.ptr( i + 1 ); ptr += m.elemSize() )
                                        { 
                                            if( ::memcmp( ptr, &zero_pixel[0], zero_pixel.size() ) != 0 ) { ++counts[i]; }
                                        }
                                    }
                                } );
            return std::accumulate( counts.begin(), counts.end(), 0 );
        }
        const uchar* ptr;
        
    private:
        std::pair< boost::optional< double >, boost::optional< double > > ratio_;
        std::pair< boost::optional< unsigned int >, boost::optional< unsigned int > > size_;
        bool empty_;
        bool keep_counting_( unsigned int count ) const
        {
            if( size_.second ) { return *size_.second < count; }
            return size_.first && ( count < *size_.first );
        }
};

} // namespace grep {

namespace thin {

class keep
{
    public:
        keep( double rate, bool deterministic )
            : rate_( rate )
            , deterministic_( deterministic )
            , distribution_( 0, 1 )
            , random_( generator_, distribution_ )
            , size_( 1.0e+9 )
            , step_( 0 )
            , count_( 0 )
        {
        }
        
        operator bool() { return deterministic_ ? deterministic_impl_() : random_() < rate_; }
        
    private:
        double rate_;
        bool deterministic_;
        boost::mt19937 generator_;
        boost::uniform_real<> distribution_;
        boost::variate_generator< boost::mt19937&, boost::uniform_real<> > random_;
        comma::uint64 size_;
        comma::uint64 step_;
        comma::uint64 count_;
        bool deterministic_impl_()
        {
            ++count_;
            if( count_ < ( step_ + 1 ) / rate_ ) { return false; }
            ++step_;
            if( step_ == size_ ) { count_ = 0; step_ = 0; }
            return true;
        }
};
    
} // namespace thin {
    
} } } // namespace snark { namespace imaging { namespace operations {

static snark::cv_mat::serialization::options handle_fields_and_format( const comma::csv::options& csv, snark::cv_mat::serialization::options input_options )
{
    if( !csv.fields.empty() && !input_options.fields.empty() ) { COMMA_THROW(comma::exception, "cv-calc: please set fields in --fields or --input, not both"); }
    if( csv.binary() && !input_options.format.elements().empty() ) { COMMA_THROW(comma::exception, "cv-calc: please set binary format in --binary or --input, not both"); }
    if( !csv.fields.empty() && input_options.fields.empty() ) { input_options.fields = csv.fields; }
    if( csv.binary() && input_options.format.string().empty() ) { input_options.format = csv.format(); }
    return input_options;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        csv.full_xpath = true;
        verbose = options.exists("--verbose,-v");
        //std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields,--input-format,--output-fields,--output-format,--show-partial", "--fields,--binary,--input,--output,--strides,--padding,--shape,--size,--kernel");
        std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--header-fields,--header-format,--output-fields,--output-format,--exit-on-stability,--crop,--no-discard,--show-partial,--permissive,--deterministic", "-.*");
        if( ops.empty() ) { std::cerr << name << "please specify an operation." << std::endl; return 1;  }
        if( ops.size() > 1 ) { std::cerr << name << "please specify only one operation, got " << comma::join( ops, ' ' ) << std::endl; return 1; }
        std::string operation = ops.front();
        const snark::cv_mat::serialization::options input_parsed = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value< std::string >( "--input", "" ) );
        snark::cv_mat::serialization::options input_options = handle_fields_and_format(csv, input_parsed );
        std::string output_options_string = options.value< std::string >( "--output", "" );
        snark::cv_mat::serialization::options output_options = output_options_string.empty() ? input_options : comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( output_options_string );
        if( input_options.no_header && !output_options.fields.empty() && input_options.fields != output_options.fields )
        {
            if( output_options.fields != snark::cv_mat::serialization::header::default_fields() ) { std::cerr << "cv-calc: when --input has no-header option, --output fields can only be fields=" << snark::cv_mat::serialization::header::default_fields() << ", got: " << output_options.fields << std::endl; return 1; }
        }
        else
        { 
            if( !output_options.fields.empty() && input_options.fields != output_options.fields ) { std::cerr << "cv-calc: customised output header fields not supported (todo); got: input fields: \"" << input_options.fields << "\" output fields: \"" << output_options.fields << "\"" << std::endl; return 1; }
        }
        if( output_options.fields.empty() ) { output_options.fields = input_options.fields; } // output fields and format will be empty when the user specifies only --output no-header or --output header-only
        if( !output_options.format.elements().empty() && input_options.format.string() != output_options.format.string() ) { std::cerr << "cv-calc: customised output header format not supported (todo); got: input format: \"" << input_options.format.string() << "\" output format: \"" << output_options.format.string() << "\"" << std::endl; return 1; }
        if( output_options.format.elements().empty() ) { output_options.format = input_options.format; };
        if( operation == "draw" )
        {
            snark::imaging::operations::draw::shapes sample( options );
            input_options.fields = comma::join( comma::csv::names< snark::imaging::operations::draw::shapes >( input_options.fields, true, sample ), ',' );
            output_options.fields = comma::join( comma::csv::names< snark::imaging::operations::draw::shapes >( output_options.fields, true, sample ), ',' );
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            comma::csv::binary< snark::imaging::operations::draw::shapes > binary( csv, sample ); // todo: sample seems not to take effect; debug
            while( std::cin.good() )
            {
                auto p = input_serialization.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
                if( p.second.empty() ) { break; }
                snark::imaging::operations::draw::shapes shape = sample; // todo: quick and dirty; tear down, once todo above is sorted
                binary.get( shape, &input_serialization.header_buffer()[0] );
                shape.draw( p.second );
                output_serialization.write_to_stdout( p, csv.flush );
            }
            return 0;
        }
        if( operation == "grep" )
        {
            // Need to be created inside, some operation (roi) has other default fields. If not using --binary also requires --fields
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            snark::imaging::operations::grep::non_zero non_zero( options.value< std::string >( "--non-zero", "" ) );
            const std::vector< snark::cv_mat::filter >& filters = snark::cv_mat::filters::make( options.value< std::string >( "--filter,--filters", "" ) );
            if( !non_zero && !filters.empty() ) { std::cerr << "cv-calc: grep: warning: --filters specified, but --non-zero is not; --filters will have no effect" << std::endl; }
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< boost::posix_time::ptime, cv::Mat > p = input_serialization.read< boost::posix_time::ptime >( std::cin );
                if( p.second.empty() ) { return 0; }
                std::pair< boost::posix_time::ptime, cv::Mat > filtered;
                if( filters.empty() ) { filtered = p; } else { p.second.copyTo( filtered.second ); }
                for( auto& filter: filters ) { filtered = filter( filtered ); }
                non_zero.size( filtered.second.rows * filtered.second.cols );
                if( non_zero.keep( filtered.second ) ) { output_serialization.write_to_stdout( p ); }
                std::cout.flush();
            }
            return 0;
        }
        if( operation == "life" )
        {
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            double procreation_threshold = options.value( "--procreation-threshold,--procreation", 3.0 );
            double stability_threshold = options.value( "--stability-threshold,--stability", 4.0 );
            double step = options.value( "--step", 1.0 );
            bool exit_on_stability = options.exists( "--exit-on-stability" );
            snark::cv_mat::impl::life< boost::posix_time::ptime > life( procreation_threshold, stability_threshold, step, exit_on_stability );
            auto iterations = options.optional< unsigned int >( "--iterations,-i" );
            std::pair< boost::posix_time::ptime, cv::Mat > p = input_serialization.read< boost::posix_time::ptime >( std::cin );
            if( p.second.empty() ) { return 0; }
            for( unsigned int i = 0; ( !iterations || i < *iterations ) && std::cout.good(); ++i )
            {
                output_serialization.write_to_stdout( life( p ) ); // todo: decouple step from output
                std::cout.flush();
            }
            return 0;
        }
        if( operation == "mean" )
        {
            if( options.exists("--output-fields") ) { std::cout << "t,rows,cols,type,mean,count" << std::endl;  exit(0); }
            if( options.exists("--output-format") ) { std::cout << "t,3ui,d,ui" << std::endl;  exit(0); }
            auto threshold = options.optional< double >("--threshold");
            snark::cv_mat::serialization serialization( input_options );
            while( std::cin.good() && !std::cin.eof() )
            {
                
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
                if( p.second.empty() ) { return 0; }
                
                cv::Mat mask;
                comma::uint32 count = p.second.rows * p.second.cols;
                if( threshold ) 
                { 
                    cv::threshold(p.second, mask, *threshold, 255, cv::THRESH_BINARY); 
                    count = cv::countNonZero(mask);
                }
                
                cv::Scalar mean = cv::mean( p.second, !threshold ? cv::noArray() : mask );
                
                std::cout.write( &serialization.header_buffer()[0], serialization.header_buffer().size() );
                for( int i = 0; i < p.second.channels(); ++i ) { std::cout.write( reinterpret_cast< char* >( &mean[i] ), sizeof( double ) ); std::cout.write( reinterpret_cast< char* >( &count ), sizeof( comma::uint32 ) ); }
                std::cout.flush();
            }
            return 0;
        }
        if( operation == "stride" )
        {
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            const std::vector< std::string >& strides_vector = comma::split( options.value< std::string >( "--strides", "1,1" ), ',' );
            if( strides_vector.size() != 2 ) { std::cerr << "cv-calc: stride: expected strides as <x>,<y>, got: \"" << options.value< std::string >( "--strides" ) << std::endl; return 1; }
            std::pair< unsigned int, unsigned int > strides( boost::lexical_cast< unsigned int >( strides_vector[0] ), boost::lexical_cast< unsigned int >( strides_vector[1] ) );
            const std::vector< std::string >& shape_vector = comma::split( options.value< std::string >( "--shape,--size,--kernel" ), ',' );
            if( shape_vector.size() != 2 ) { std::cerr << "cv-calc: stride: expected shape as <x>,<y>, got: \"" << options.value< std::string >( "--shape,--size,--kernel" ) << std::endl; return 1; }
            std::pair< unsigned int, unsigned int > shape( boost::lexical_cast< unsigned int >( shape_vector[0] ), boost::lexical_cast< unsigned int >( shape_vector[1] ) );
            unsigned int shape_size = shape.first * shape.second;
            struct padding_types { enum values { same, valid }; };
            std::string padding_string = options.value< std::string >( "--padding", "valid" );
            padding_types::values padding = padding_types::same;
            if( padding_string == "same" || padding_string == "SAME" ) { padding = padding_types::same; std::cerr << "cv-calc: stride: padding 'same' not implemented; please use --padding=valid" << std::endl; return 1; }
            else if( padding_string == "valid" || padding_string == "VALID" ) { padding = padding_types::valid; }
            else { std::cerr << "cv-calc: stride: expected padding type, got: \"" << padding_string << "\"" << std::endl; return 1; }
            snark::imaging::operations::grep::non_zero non_zero( options.value< std::string >( "--non-zero", "" ) );
            typedef snark::cv_mat::serialization::header::buffer_t first_t; // typedef boost::posix_time::ptime first_t;
            typedef std::pair< first_t, cv::Mat > pair_t;
            typedef snark::cv_mat::filter_with_header filter_t; // typedef snark::cv_mat::filter filter_t;
            typedef snark::cv_mat::filters_with_header filters_t; // typedef snark::cv_mat::filters filters_t;
            const comma::csv::binary< snark::cv_mat::serialization::header >* binary = input_serialization.header_binary();
            auto get_timestamp_from_header = [&]( const snark::cv_mat::serialization::header::buffer_t& h )->boost::posix_time::ptime
            {
                if( h.empty() || !binary ) { return boost::posix_time::not_a_date_time; }
                snark::cv_mat::serialization::header d;
                return binary->get( d, &h[0] ).timestamp;
            };
            const std::vector< filter_t >& filters = filters_t::make( options.value< std::string >( "--filter,--filters", "" ), get_timestamp_from_header );
            if( !non_zero && !filters.empty() ) { std::cerr << "cv-calc: stride: warning: --filters specified, but --non-zero is not; --filters will have no effect" << std::endl; }
            while( std::cin.good() && !std::cin.eof() )
            {
                pair_t p = input_serialization.read< first_t >( std::cin );
                if( p.second.empty() ) { return 0; }
                pair_t filtered;
                if( !filters.empty() )
                {
                    p.second.copyTo( filtered.second );
                    for( auto& filter: filters ) { filtered = filter( filtered ); }
                    if( filtered.second.rows != p.second.rows || filtered.second.cols != p.second.cols ) { std::cerr << "cv-calc: stride: expected original and filtered images of the same size, got " << p.second.rows << "," << p.second.cols << " vs " << filtered.second.rows << "," << filtered.second.cols << std::endl; return 1; }
                }
                switch( padding )
                {
                    case padding_types::same: // todo
                        break;
                    case padding_types::valid:
                    {
                        if( p.second.cols < int( shape.first ) || p.second.rows < int( shape.second ) ) { std::cerr << "cv-calc: stride: expected image greater than rows: " << shape.second << " cols: " << shape.first << "; got rows: " << p.second.rows << " cols: " << p.second.cols << std::endl; return 1; }
                        pair_t q;
                        q.first = p.first;
                        for( unsigned int j = 0; j < ( p.second.rows + 1 - shape.second ); j += strides.second )
                        {
                            for( unsigned int i = 0; i < ( p.second.cols + 1 - shape.first ); i += strides.first )
                            {
                                if( !filtered.second.empty() )
                                {
                                    filtered.second( cv::Rect( i, j, shape.first, shape.second ) ).copyTo( q.second );
                                    non_zero.size( shape_size );
                                    if( !non_zero.keep( q.second ) ) { continue; }
                                }
                                p.second( cv::Rect( i, j, shape.first, shape.second ) ).copyTo( q.second );
                                output_serialization.write_to_stdout( q, csv.flush );
                            }
                        }
                        break;
                    }
                }
            }
            return 0;
        }
        if( operation == "thin" )
        {
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            options.assert_mutually_exclusive( "--rate,--to-fps,--fps" );
            options.assert_mutually_exclusive( "--deterministic,--to-fps,--fps" );
            if( !options.exists( "--rate" ) && !options.exists( "--to-fps,--fps" ) ) { std::cerr << "cv-calc: thin: please specify either --rate or --to-fps" << std::endl; }
            bool deterministic = options.exists( "--to-fps,--fps" ) || options.exists( "--deterministic" );
            double rate = options.exists( "--rate" ) ? options.value< double >( "--rate" ) : options.value< double >( "--to-fps,--fps" ) / options.value< double >( "--from-fps" );
            snark::imaging::operations::thin::keep keep( rate, deterministic );
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = input_serialization.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
                if( p.second.empty() ) { return 0; }
                if( keep ) { output_serialization.write_to_stdout( p ); std::cout.flush(); }
            }
            return 0;
        }
        if( operation == "unstride-positions" ) // TODO move to another utility since it doesn't operate on an image ?
        {
            stride_positions_t input( options );
            positions_t output( options );

            if( options.exists("--input-fields") ) { std::cout << comma::join( comma::csv::names< stride_positions_t >( true, input ), ',' ) << std::endl; return 0; }
            if( options.exists("--input-format") ) { std::cout << comma::csv::format::value< stride_positions_t >( "", true, input ) << std::endl; return 0; }
            if( options.exists("--output-fields") ) { std::cout << comma::join( comma::csv::names< positions_t >( true, output ), ',' ) << std::endl; return 0; }
            if( options.exists("--output-format") ) { std::cout << comma::csv::format::value< positions_t >( "", true, output ) << std::endl; return 0; }

            const std::vector< std::string >& unstrided_vector = comma::split( options.value< std::string >( "--unstrided-size,--unstrided" ), ',' );
            if( unstrided_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected --unstrided-size as <width>,<height>, got: \"" << options.value< std::string >( "--unstrided-size,--unstrided" ) << std::endl; return 1; }
            cv::Point2i unstrided( boost::lexical_cast< unsigned int >( unstrided_vector[0] ), boost::lexical_cast< unsigned int >( unstrided_vector[1] ) );
            const std::vector< std::string >& strides_vector = comma::split( options.value< std::string >( "--strides", "1,1" ), ',' );
            if( strides_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected strides as <x>,<y>, got: \"" << options.value< std::string >( "--strides" ) << std::endl; return 1; }
            cv::Point2i strides( boost::lexical_cast< unsigned int >( strides_vector[0] ), boost::lexical_cast< unsigned int >( strides_vector[1] ) );
            const std::vector< std::string >& shape_vector = comma::split( options.value< std::string >( "--shape,--size,--kernel" ), ',' );
            if( shape_vector.size() != 2 ) { std::cerr << "cv-calc: unstride-positions: expected shape as <x>,<y>, got: \"" << options.value< std::string >( "--shape,--size,--kernel" ) << std::endl; return 1; }
            cv::Point2i shape( boost::lexical_cast< unsigned int >( shape_vector[0] ), boost::lexical_cast< unsigned int >( shape_vector[1] ) );

            // TODO --padding
            unsigned int stride_rows = ( unstrided.y - shape.y ) / strides.y + 1;
            unsigned int stride_cols = ( unstrided.x - shape.x ) / strides.x + 1;
            unsigned int num_strides = stride_rows * stride_cols;
            if( verbose ) { std::cerr << name << "unstride-positions: stride rows: " << stride_rows << ", stride cols: " << stride_cols << std::endl; }

            comma::csv::options icsv( options );
            icsv.full_xpath = true;
            comma::csv::input_stream< stride_positions_t > is( std::cin, icsv, input );
            comma::csv::options ocsv;
            ocsv.fields = comma::join( comma::csv::names< positions_t >( "positions", true, output ), ',' );
            ocsv.flush = true;
            ocsv.full_xpath = true;
            if( options.exists( "--binary" ) ) { ocsv.format( comma::csv::format::value< positions_t >( ocsv.fields, true, output ) ); }
            comma::csv::output_stream< positions_t > os( std::cout, ocsv, output );
            comma::csv::tied< stride_positions_t, positions_t > tied( is, os );

            while( is.ready() || std::cin.good() )
            {
                const stride_positions_t* p = is.read();
                if( !p ) { break; }
                if( p->index >= num_strides ) { COMMA_THROW( comma::exception, "invalid stride index: " << p->index << "; expected index < " << num_strides ); }
                unsigned int stride_col = p->index % stride_rows;
                unsigned int stride_row = p->index / stride_rows;
                cv::Point2d offset( stride_col * strides.x, stride_row * strides.y );
                for( unsigned int i = 0; i < p->positions.size(); ++i ) { output.positions[i] = p->positions[i] + offset; }
                tied.append( output );
            }
            return 0;
        }
        if( operation == "header" )
        {
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            
            if( options.exists("--header-fields") ) { std::cout << "t,rows,cols,type" << std::endl;  exit(0); }
            if( options.exists("--header-format") ) { std::cout << "t,3ui" << std::endl;  exit(0); }
            if( verbose ) { std::cerr << name << "fields: " << input_options.fields << std::endl; std::cerr << name << "format: " << input_options.format.string() << std::endl; }            
            if( options.exists("--output-fields") ) { std::cout << "rows,cols,type" << std::endl;  return 0; }
            
            snark::cv_mat::serialization serialization( input_options );
            std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
            if( p.second.empty() ) { std::cerr << name << "failed to read input stream" << std::endl; return 1; }
            comma::csv::options out;
            out.fields = "rows,cols,type";
            comma::csv::output_stream< snark::cv_mat::serialization::header > ascii( std::cout, out );
            ascii.write( serialization.get_header( &serialization.header_buffer()[0] ) );
            return 0;
        }
        if( operation == "format" )
        {
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            if( options.exists("--header-fields") ) { std::cout << "t,rows,cols,type" << std::endl;  exit(0); }
            if( options.exists("--header-format") ) { std::cout << "t,3ui" << std::endl;  exit(0); }
            if( verbose ) { std::cerr << name << "fields: " << input_options.fields << std::endl; std::cerr << name << "format: " << input_options.format.string() << std::endl; }            
            
            snark::cv_mat::serialization serialization( input_options );
            std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
            if( p.second.empty() ) { std::cerr << name << "failed to read input stream" << std::endl; return 1; }
            snark::cv_mat::serialization::header header = serialization.get_header( &serialization.header_buffer()[0] );
            comma::csv::format format = input_options.format.elements().empty() ? comma::csv::format("t,3ui") : input_options.format ;
            format += "s[" + boost::lexical_cast<std::string>( comma::uint64(header.rows) * header.cols * p.second.elemSize() )  + "]";
            std::cout << format.string() << std::endl;
            return 0;
        }
        if( operation == "roi" )
        {
            // TODO: in order to do this, extents need to be built into serialisation::options
            if( input_options.no_header ) { std::cerr << "cv-calc: --input with no-header cannot be used with 'roi' operation, as roi extents is passed in with the header" << std::endl; return 1; }
            if( options.exists("--header-fields") ) { std::cout << comma::join( comma::csv::names<extents>(), ',' ) << "," << "t,rows,cols,type" << std::endl;  exit(0); }
            if( options.exists("--header-format") ) { std::cout << comma::csv::format::value<extents>() << "," << "t,3ui" << std::endl;  exit(0); }
            if( input_options.fields.empty() ) { input_options.fields = "min/x,min/y,max/x,max/y,t,rows,cols,type"; }
            if( input_options.format.elements().empty() ) { input_options.format = comma::csv::format( "4i,t,3ui" ); }
            if( output_options_string.empty() ) { output_options = input_options; }
            if( verbose ) { std::cerr << name << "fields: " << input_options.fields << std::endl; std::cerr << name << "format: " << input_options.format.string() << std::endl; }            
            
            snark::cv_mat::serialization input_serialization( input_options );
            snark::cv_mat::serialization output_serialization( output_options );
            
            options.assert_mutually_exclusive( "--crop,--no-discard" );
            csv.fields = input_options.fields;
            csv.format( input_options.format );
            comma::csv::binary< ::extents > binary( csv );
            bool crop = options.exists( "--crop" );
            bool flush = options.exists("--flush");
            bool permissive = options.exists("--show-partial,--permissive");
            bool no_discard = options.exists( "--no-discard" );
            ::extents ext;
            cv::Mat mask;
            comma::uint64 count = 0;
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = input_serialization.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
                if( p.second.empty() ) { break; }
                cv::Mat& mat = p.second;
                ++count;
                binary.get( ext, &input_serialization.header_buffer()[0] );
                if( mask.rows != mat.rows || mask.cols != mat.cols ) { mask = cv::Mat::ones( mat.rows, mat.cols, CV_8U ); }
                if( ext.max.x < 0 || ext.min.x >= mat.cols || ext.max.y < 0 || ext.min.y >= mat.rows )
                {
                    if( no_discard )
                    {
                        mat.setTo( cv::Scalar(0) );
                        output_serialization.write_to_stdout( p, flush );
                    }
                    continue;
                }
                if( permissive )
                {
                    ext.min.x = std::max( ext.min.x, 0 );
                    ext.min.y = std::max( ext.min.y, 0 );
                    ext.max.x = std::min( ext.max.x, mat.cols );
                    ext.max.y = std::min( ext.max.y, mat.rows );
                }                
                int width = ext.max.x - ext.min.x;
                int height = ext.max.y - ext.min.y;
                if( width < 0 || height < 0 ) { std::cerr << name << "roi's width and height can not be negative; failed on image/frame number " << count << ", min: " << ext.min << ", max: " << ext.max << ", width: " << width << ", height: " << height << std::endl; return 1; }
                if( ext.min.x >= 0 && ext.min.y >=0 && ext.max.x <= mat.cols && ext.max.y <= mat.rows )
                {
                    if( crop )
                    {
                        cv::Mat cropped;
                        p.second( cv::Rect( ext.min.x, ext.min.y, width, height ) ).copyTo( cropped );
                        output_serialization.write_to_stdout( std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat >( p.first, cropped ) );
                    }
                    else
                    {
                        mask( cv::Rect( ext.min.x, ext.min.y, width , height ) ) = cv::Scalar(0);
                        mat.setTo( cv::Scalar(0), mask );
                        mask( cv::Rect( ext.min.x, ext.min.y, width , height ) ) = cv::Scalar(1);
                        output_serialization.write_to_stdout( p, flush );
                    }
                }
            }
            return 0;
        }
        std::cerr << name << " unknown operation: " << operation << std::endl;
    }
    catch( std::exception& ex ) { std::cerr << "cv-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "cv-calc: unknown exception" << std::endl; }
    return 1;
}

