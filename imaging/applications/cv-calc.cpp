// This file is part of Ark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// Ark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Ark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with Ark. If not, see <http://www.gnu.org/licenses/>.

#include <algorithm>
#include <memory>
#include <opencv2/core/core.hpp>
#include <tbb/parallel_for.h>
#include <comma/visiting/traits.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/math/interval.h>
#include <comma/name_value/parser.h>
#include "../../imaging/cv_mat/filters.h"
#include "../../imaging/cv_mat/serialization.h"

const char* name = "cv-calc: ";
const char* default_input_fields = "min/x,min/y,max/x,max/y,t,rows,cols,type";

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
    std::cerr << "    format" << std::endl;
    std::cerr << "        output header and data format string in ascii" << std::endl;
    std::cerr << "    grep" << std::endl;
    std::cerr << "        output only images that satisfy conditions" << std::endl;
    std::cerr << "    header" << std::endl;
    std::cerr << "        output header information in ascii csv" << std::endl;
    std::cerr << "    mean" << std::endl;
    std::cerr << "        output image means for all image channels appended to image header" << std::endl;
    std::cerr << "    roi" << std::endl;
    std::cerr << "        given cv image data associated with a region of interest, set everything outside the region of interest to zero" << std::endl;
    std::cerr << "    stride" << std::endl;
    std::cerr << "        stride through the image, output images of kernel size for each pixel" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --binary=[<format>]: binary format of header; default: operation-dependent, use --header-format" << std::endl;
    std::cerr << "    --fields=<fields>; default: operation-dependent, use --header-fields" << std::endl;
    std::cerr << "    --flush; flush after every image" << std::endl;
    std::cerr << "    --input=<options>; default values for image header; e.g. --input=\"rows=1000;cols=500;type=ub\"" << std::endl;
    std::cerr << "    --input-fields; output header fields and exit" << std::endl;
    std::cerr << "    --input-format; output header format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "serialization options" << std::endl;
    if( verbose ) { std::cerr << snark::cv_mat::serialization::options::usage() << std::endl; } else { std::cerr << "    run --help --verbose for more details..." << std::endl; }
    std::cerr << std::endl;
    std::cerr << "operation options" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    grep" << std::endl;
    std::cerr << "        --filter,--filters=[<filters>]; todo; apply --non-zero logic to the image with filters applied, not to image itself" << std::endl;
    std::cerr << "                                        run cv-cat --help --verbose for filters available" << std::endl;
    std::cerr << "        --non-zero=[<what>]; output only images that have non-zero pixels" << std::endl;
    std::cerr << "            <what>" << std::endl;
    std::cerr << "                all: output only images with all pixels non-zero" << std::endl;
    std::cerr << "                none: output only images with all pixels zero, makes sense only if --mask given" << std::endl;
    std::cerr << "                some: output only images with some pixels non-zero" << std::endl;
    std::cerr << "                <ratio>: output only images with a given ration of non-zero pixels: todo?" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    roi" << std::endl;
    std::cerr << "        --show-partial; by default no partial roi is shown in image. Use option to change behaviour." << std::endl;
    std::cerr << "        --discard; discards frames where the roi is not seen." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    stride" << std::endl;
    std::cerr << "        --input=[<options>]; input options; run cv-cat --help --verbose for details" << std::endl;
    std::cerr << "        --output=[<options>]; output options; run cv-cat --help --verbose for details" << std::endl;
    std::cerr << "        --padding=[<padding>]; padding, 'same' or 'valid' (see e.g. tensorflow for the meaning); default: valid" << std::endl;
    std::cerr << "        --shape,--kernel,--size=<x>,<y>; image size" << std::endl;
    std::cerr << "        --strides=[<x>,<y>]; stride size; default: 1,1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "  header" << std::endl;
    std::cerr << "      cat data.bin | cv-calc header" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  format" << std::endl;
    std::cerr << "      cat data.bin | cv-calc format" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  roi" << std::endl;
    std::cerr << "      Setting everything but the roi rectangle to 0 for all images" << std::endl;
    std::cerr << "      ROI fields must be pre-pended. This roi is is a square of (100,100) to (300,300)" << std::endl;
    std::cerr << "      Given a cv-cat image stream with format 't,3ui,s[1572864]'." << std::endl;
    std::cerr << std::endl;
    std::cerr << "      cat data.bin | csv-paste \"value=100,100,300,300;binary=4i\" \"-;binary=t,3ui,s[1572864]\" \\" << std::endl;
    std::cerr << "          | cv-calc roi -v | csv-bin-cut '4i,t,3ui,s[1572864]' --fields 5-9 >output.bin" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      Explicity specifying fields. Image payload data field is not specified for cv-calc, not set for --binary either" << std::endl;
    std::cerr << "      The user must explcitly list all four roi fields. Using 'min,max' is not possible." << std::endl;
    std::cerr << std::endl;
    std::cerr << "      cat data.bin | csv-paste \"value=100,100,999,300,300;binary=5i\" \"-;binary=t,3ui,s[1572864]\" \\" << std::endl;
    std::cerr << "          | cv-calc roi --fields min/x,min/y,,max/x,max/y,t,rows,cols,type --binary '5i,t,3ui' \\" << std::endl;
    std::cerr << "          | csv-bin-cut '5i,t,3ui,s[1572864]' --fields 6-10 >output.bin" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct extents
{
    cv::Point2i min;
    cv::Point2i max;
};

namespace comma { namespace visiting {
    
template <> struct traits< cv::Point2i >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, cv::Point2i& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, const cv::Point2i& p, Visitor& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
    }
};

template <> struct traits< ::extents >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ::extents& p, Visitor& v )
    {
        v.apply( "min", p.min );
        v.apply( "max", p.max );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ::extents& p, Visitor& v )
    {
        v.apply( "min", p.min );
        v.apply( "max", p.max );
    }
};

} } // namespace comma { namespace visiting {

static bool verbose = false;

int main( int ac, char** av )
{
    
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options csv( options );
        csv.full_xpath = true;
        verbose = options.exists("--verbose,-v");
        //std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields,--input-format,--output-fields,--output-format,--show-partial,--discard", "--fields,--binary,--input,--output,--strides,--padding,--shape,--size,--kernel");
        std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields,--input-format,--output-fields,--output-format,--show-partial,--discard", "-.*");
        if( ops.empty() ) { std::cerr << name << "please specify an operation." << std::endl; return 1;  }
        if( ops.size() > 1 ) { std::cerr << name << "please specify only one operation, got " << comma::join( ops, ' ' ) << std::endl; return 1; }
        std::string operation = ops.front();
        snark::cv_mat::serialization::options input_options = comma::name_value::parser( ';', '=' ).get< snark::cv_mat::serialization::options >( options.value< std::string >( "--input", "" ) );
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
        snark::cv_mat::serialization input_serialization( input_options );
        snark::cv_mat::serialization output_serialization( output_options );
        if( operation == "grep" )
        {
            class non_zero_t
            { 
                public:
                    non_zero_t( const std::string s ) : min_size_( 0 )
                    {
                        if( s.empty() ) { return; }
                        if( s == "all" ) { how_ = all; return; }
                        if( s == "none" ) { how_ = none; return; }
                        if( s == "some" ) { how_ = some; return; }
                        how_ = ratio;
                        ratio_ = boost::lexical_cast< double >( s );
                    }
                    operator bool() const { return static_cast< bool >( how_ ); }
                    void min_size( unsigned int image_size ) { if( how_ ) { min_size_ = *how_ == all ? image_size : *how_ == some ? 1 : *how_ == ratio ? image_size * ratio : 0; } }
                    bool keep( unsigned int count ) const
                    { 
                        if( !how_ ) { return true; }
                        switch( *how_ )
                        {
                            case none: return count == 0;
                            default: return count >= min_size_;
                        }
                    }
                    bool keep_counting( unsigned int count ) const
                    {
                        if( !how_ ) { return false; }
                        switch( *how_ )
                        {
                            case none: return count == 0;
                            default: return count < min_size_;
                        }
                    }
                private:
                    enum how_t_ { all, none, some, ratio };
                    boost::optional< how_t_ > how_;
                    double ratio_;
                    unsigned int min_size_;

            };
            non_zero_t non_zero( options.value< std::string >( "--non-zero", "" ) );
            const std::vector< snark::cv_mat::filter >& filters = snark::cv_mat::filters::make( options.value< std::string >( "--filter,--filters", "" ) );
            if( !non_zero && !filters.empty() ) { std::cerr << "cv-calc: warning: --filters specified, but --non-zero is not; --filters will have no effect" << std::endl; }
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< boost::posix_time::ptime, cv::Mat > p = input_serialization.read< boost::posix_time::ptime >( std::cin );
                if( p.second.empty() ) { return 0; }
                for( auto& filter: filters ) { p = filter( p ); }
                non_zero.min_size( p.second.rows * p.second.cols );
                static std::vector< char > zero_pixel( p.second.elemSize(), 0 );
                std::vector< unsigned int > counts( p.second.rows, 0 );
                tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, p.second.rows )
                                 , [&]( const tbb::blocked_range< std::size_t >& r )
                                   {
                                       for( unsigned int i = r.begin(); i < r.end() && non_zero.keep_counting( counts[i] ); ++i )
                                       {
                                           for( const unsigned char* ptr = p.second.ptr( i ); ptr < p.second.ptr( i + 1 ); ptr += p.second.elemSize() )
                                           { 
                                               if( ::memcmp( ptr, &zero_pixel[0], zero_pixel.size() ) != 0 ) { ++counts[i]; }
                                           }
                                       }
                                   } );
                unsigned int count = std::accumulate( counts.begin(), counts.end(), 0 );
                if( non_zero.keep( count ) ) { output_serialization.write_to_stdout( p ); }
                std::cout.flush();
            }
            return 0;
        }
        if( operation == "mean" )
        {
            if( csv.fields.empty() ) { csv.fields = "t,rows,cols,type"; }
            if( !csv.binary() ) { csv.format( "t,3ui" ); }
            snark::cv_mat::serialization serialization( csv.fields, csv.format() );
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
                if( p.second.empty() ) { return 0; }
                cv::Scalar mean = cv::mean( p.second );
                std::cout.write( &serialization.header_buffer()[0], serialization.header_buffer().size() );
                for( int i = 0; i < p.second.channels(); ++i ) { std::cout.write( reinterpret_cast< char* >( &mean[i] ), sizeof( double ) ); }
                std::cout.flush();
            }
            return 0;
        }
        if( operation == "stride" )
        {
            const std::vector< std::string >& strides_vector = comma::split( options.value< std::string >( "--strides", "1,1" ), ',' );
            if( strides_vector.size() != 2 ) { std::cerr << "cv-calc: stride: expected strides as <x>,<y>, got: \"" << options.value< std::string >( "--strides" ) << std::endl; return 1; }
            std::pair< unsigned int, unsigned int > strides( boost::lexical_cast< unsigned int >( strides_vector[0] ), boost::lexical_cast< unsigned int >( strides_vector[1] ) );
            const std::vector< std::string >& shape_vector = comma::split( options.value< std::string >( "--shape,--size,--kernel" ), ',' );
            if( shape_vector.size() != 2 ) { std::cerr << "cv-calc: stride: expected shape as <x>,<y>, got: \"" << options.value< std::string >( "--shape,--size,--kernel" ) << std::endl; return 1; }
            std::pair< unsigned int, unsigned int > shape( boost::lexical_cast< unsigned int >( shape_vector[0] ), boost::lexical_cast< unsigned int >( shape_vector[1] ) );
            struct padding_types { enum values { same, valid }; };
            std::string padding_string = options.value< std::string >( "--padding", "valid" );
            padding_types::values padding = padding_types::same;
            if( padding_string == "same" || padding_string == "SAME" ) { padding = padding_types::same; std::cerr << "cv-calc: stride: padding 'same' not implemented; please use --padding=valid" << std::endl; return 1; }
            else if( padding_string == "valid" || padding_string == "VALID" ) { padding = padding_types::valid; }
            else { std::cerr << "cv-calc: stride: expected padding type, got: \"" << padding_string << "\"" << std::endl; return 1; }
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = input_serialization.read< snark::cv_mat::serialization::header::buffer_t >( std::cin );
                if( p.second.empty() ) { return 0; }
                switch( padding )
                {
                    case padding_types::same: // todo
                        break;
                    case padding_types::valid:
                    {
                        if( p.second.cols < int( shape.first ) || p.second.rows < int( shape.second ) ) { std::cerr << "cv-calc: expected image greater than rows: " << shape.second << " cols: " << shape.first << "; got rows: " << p.second.rows << " cols: " << p.second.cols << std::endl; return 1; }
                        std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > q;
                        q.first = p.first;
                        for( unsigned int i = 0; i < ( p.second.cols + 1 - shape.first ); i += strides.first )
                        {
                            for( unsigned int j = 0; j < ( p.second.rows + 1 - shape.second ); j += strides.second )
                            {
                                p.second( cv::Rect( i, j, shape.first, shape.second ) ).copyTo( q.second );
                                output_serialization.write_to_stdout( q );
                            }
                        }
                        break;
                    }
                }
                std::cout.flush();
            }
            return 0;
        }
        if( operation == "header" || operation == "format" )
        {
            if( csv.fields.empty() ) { csv.fields = "t,rows,cols,type" ; }
            if( !csv.binary() ) { csv.format("t,3ui"); }
            
            if( options.exists("--input-fields") ) { std::cout << "t,rows,cols,type" << std::endl;  exit(0); }
            if( options.exists("--input-format") ) { std::cout << "t,3ui" << std::endl;  exit(0); }
        }
        else if( operation == "roi" )
        {
            if( csv.fields.empty() ) { csv.fields = default_input_fields ; }
            if( !csv.binary() ) { csv.format("4i,t,3ui"); }
            if( options.exists("--input-fields,--output-fields") ) { std::cout << comma::join( comma::csv::names<extents>(), ',' ) << "," << "t,rows,cols,type" << std::endl;  exit(0); }
            if( options.exists("--input-format,--output-format") ) { std::cout << comma::csv::format::value<extents>() << "," << "t,3ui" << std::endl;  exit(0); }
        }
        
        if( verbose )
        {
            std::cerr << name << "fields: " << csv.fields << std::endl;
            std::cerr << name << "format: " << csv.format().string() << std::endl;
        }
            
        snark::cv_mat::serialization serialization( csv.fields, csv.format() ); // todo?
        
        if( operation == "header" )
        {
            if( options.exists("--output-fields") ) { std::cout << "rows,cols,type" << std::endl;  exit(0); }
            
            if( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
                if( p.second.empty() ) { std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
                
                comma::csv::options out;
                out.fields = "rows,cols,type";
                comma::csv::output_stream< snark::cv_mat::serialization::header > ascii( std::cout, out );
                ascii.write( serialization.get_header( &serialization.header_buffer()[0] ) );
            }
            else{ std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
        }
        else if( operation == "format" )
        {
            if( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
                if( p.second.empty() ) { std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
                
                snark::cv_mat::serialization::header header = serialization.get_header( &serialization.header_buffer()[0] );
                
                comma::csv::format format = csv.format();
                format += "s[" + boost::lexical_cast<std::string>( comma::uint64(header.rows) * header.cols * p.second.elemSize() )  + "]";
                std::cout << format.string() << std::endl;
            }
            else{ std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
        }
        else if( operation == "roi" )
        {
            comma::csv::binary< ::extents > binary( csv );
            bool flush = options.exists("--flush");
            bool show_partial = options.exists("--show-partial");
            
            if( verbose ) { std::cerr << name << "show partial: " << show_partial << std::endl; }
            
            ::extents ext;
            cv::Mat mask;
            comma::uint64 count = 0;
            while( std::cin.good() && !std::cin.eof() )
            {
                std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
                cv::Mat& mat = p.second;
                if( mat.empty() ) { break; }
                
                ++count;
                
                binary.get( ext, &serialization.header_buffer()[0] );
                if(verbose && mask.rows == 0) // Will only trigger once
                {
                    snark::cv_mat::serialization::header header = serialization.get_header( &serialization.header_buffer()[0] );
                    std::cerr << name << "min & max: " << ext.min << " " << ext.max << std::endl;
                    std::cerr << name << "rows & cols: " << header.rows << ' ' << header.cols << std::endl;
                }
                
                // If image size changed
                if( mask.rows != mat.rows || mask.cols != mat.cols ) { mask = cv::Mat::ones(mat.rows, mat.cols, CV_8U); }  // all ones, must be CV_U8 for setTo
                
                // roi not in image at all
                if( ext.max.x < 0 || ext.min.x >= mat.cols || ext.max.y < 0 || ext.min.y >= mat.rows ) { continue; }
                    
                // Clip roi to fit in the image
                if( show_partial )
                {
                    if( ext.min.x < 0 ) { ext.min.x = 0; }
                    if( ext.max.x >= mat.cols ) { ext.max.x = mat.cols-1; }
                    if( ext.min.y < 0 ) { ext.min.y = 0; }
                    if( ext.max.y >= mat.rows ) { ext.max.y = mat.rows-1; }
                }
                
                int width = ext.max.x - ext.min.x;
                int height = ext.max.y - ext.min.y;
                // Mask to set anything not in the ROI to 0
                if( width < 0 || height < 0 ) {
                    std::cerr << name << "roi's width and height can not be negative. Failed on image/frame number: " << count 
                        << ", min: " << ext.min << ", max: " << ext.max << ", width: " << width << ", height: " << height << std::endl; return 1;
                }
                
                if( ext.min.x >= 0 && ext.min.y >=0 && (ext.min.x + width < mat.cols) && (ext.min.y + height < mat.rows ) ) 
                {
                    mask( cv::Rect( ext.min.x, ext.min.y, width , height ) ) = cv::Scalar(0);
                    mat.setTo( cv::Scalar(0), mask );
                    mask( cv::Rect( ext.min.x, ext.min.y, width , height ) ) = cv::Scalar(1);
                    serialization.write_to_stdout( p, flush );
                }
            }
        }
        else { std::cerr << name << " unknown operation: " << operation << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "cv-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "cv-calc: unknown exception" << std::endl; }
    return 1;
}

