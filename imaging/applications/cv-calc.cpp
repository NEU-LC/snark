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

#include <opencv2/core/core.hpp>
#include <comma/visiting/traits.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/math/interval.h>
#include "../../imaging/cv_mat/serialization.h"
// #include "../../visiting/eigen.h"

const char* name = "cv-calc: ";
const char* default_input_fields = "min/x,min/y,max/x,max/y,t,rows,cols,type";

static void usage( bool verbose=false )
{
    std::cerr << std::endl;
    std::cerr << "Performs verious image manipulation or calculations on cv image streams." << std::endl;
    std::cerr << "usage: cat bumblebee2.bin | cv-calc <operation> [<options>] > bumblebee2_roi.bin " << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "  roi" << std::endl;
    std::cerr << "      Given cv image data associated with a region of interest, set everything outside the region of interest to zero" << std::endl;
    std::cerr << "  header" << std::endl;
    std::cerr << "      Outputs header information in ascii csv" << std::endl;
    std::cerr << "  format" << std::endl;
    std::cerr << "      Outputs header and data format string in ascii" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --binary=[<format>]: binary format of header; default: operation-dependent, use --header-format" << std::endl;
    std::cerr << "    --fields=<fields>; default: operation-dependent, use --header-fields" << std::endl;
    std::cerr << "    --flush; flush after every image" << std::endl;
    std::cerr << "    --input=<options>; default values for image header; e.g. --input=\"rows=1000;cols=500;type=ub\"" << std::endl;
    std::cerr << "    --input-fields; output header fields and exit" << std::endl;
    std::cerr << "    --input-format; output header format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  roi" << std::endl;
    std::cerr << "    --show-partial; by default no partial roi is shown in image. Use option to change behaviour." << std::endl;
    std::cerr << "    --discard; discards frames where the roi is not seen." << std::endl;
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

namespace Ark { namespace Applications {

struct extents {
    cv::Point2i min;
    cv::Point2i max;
};

} } // namespace Ark { namespace Applications {

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

template <> struct traits< Ark::Applications::extents >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, Ark::Applications::extents& p, Visitor& v )
    {
        v.apply( "min", p.min );
        v.apply( "max", p.max );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const Ark::Applications::extents& p, Visitor& v )
    {
        v.apply( "min", p.min );
        v.apply( "max", p.max );
    }
};

} } // namespace comma { namespace visiting {

typedef Ark::Applications::extents extents_t;

static bool verbose = false;

int main( int ac, char** av )
{
    comma::command_line_options options( ac, av, usage );
    comma::csv::options csv( options );
    csv.full_xpath = true;
    
    using Ark::Applications::extents;
    using snark::cv_mat::serialization;
        
    
    verbose = options.exists("--verbose,-v");
    
    std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields,--input-format,--output-fields,--output-format,--show-partial,--discard", "--fields,--binary,--input");
    if( ops.empty() ) { std::cerr << name << "please specify an operation." << std::endl; usage(false);  }
    if( ops.size() > 1 ) { std::cerr << name << "please specify only one operation, got " << comma::join( ops, ' ' ) << std::endl; usage(false); }
    std::string operation = ops.front();
    
    if( operation == "header" || operation == "format" )
    {
        if( csv.fields.empty() ) { csv.fields = "t,rows,cols,type" ; }
        if( !csv.binary() ) { csv.format("t,3ui"); }
        
        if( options.exists("--input-fields") ) { std::cout << "t,rows,cols,type" << std::endl;  exit(0); }
        if( options.exists("--input-format") ) { std::cout << "t,3ui" << std::endl;  exit(0); }
    }
    else if( operation == "mask" )
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
            comma::csv::output_stream< serialization::header > ascii( std::cout, out );
            ascii.write( serialization.get_header( serialization.header_buffer()) );
        }
        else{ std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
    }
    else if( operation == "format" )
    {
        if( std::cin.good() && !std::cin.eof() )
        {
            std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
            if( p.second.empty() ) { std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
            
            serialization::header header = serialization.get_header( serialization.header_buffer());
            
            comma::csv::format format = csv.format();
            format += "s[" + boost::lexical_cast<std::string>( comma::uint64(header.rows) * header.cols * p.second.elemSize() )  + "]";
            std::cout << format.string() << std::endl;
        }
        else{ std::cerr << name << "failed to read input stream" << std::endl; exit(1); }
    }
    else if( operation == "roi" )
    {
        
        comma::csv::binary< extents_t > binary( csv );
        bool flush = options.exists("--flush");
        bool show_partial = options.exists("--show-partial");
        
        if( verbose ) { std::cerr << name << "show partial: " << show_partial << std::endl; }
        
        extents_t ext;
        cv::Mat mask;
        comma::uint64 count = 0;
        while( std::cin.good() && !std::cin.eof() )
        {
            std::pair< snark::cv_mat::serialization::header::buffer_t, cv::Mat > p = serialization.read< snark::cv_mat::serialization::header::buffer_t >(std::cin);
            cv::Mat& mat = p.second;
            if( mat.empty() ) { break; }
            
            ++count;
            
            binary.get( ext, serialization.header_buffer() );
            if(verbose && mask.rows == 0) // Will only trigger once
            {
                serialization::header header = serialization.get_header( serialization.header_buffer());
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
            
            if( ext.min.x >= 0 && ext.min.y >=0 
                && (ext.min.x + width < mat.cols) && (ext.min.y + height < mat.rows) 
            ) 
            {
                mask( cv::Rect( ext.min.x, ext.min.y, width , height ) ) = cv::Scalar(0);
                mat.setTo( cv::Scalar(0), mask );
                mask( cv::Rect( ext.min.x, ext.min.y, width , height ) ) = cv::Scalar(1);
                serialization.write(std::cout, p, flush );
            }
        }
    }
    else { std::cerr << name << " unknown operation: " << operation << std::endl; return 1; }
    return 0;
}

