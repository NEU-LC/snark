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

const char* name = "image-color-calc: ";
const char* default_input_fields = "min/x,min/y,max/x,max/y,t,rows,cols,type";

static void usage( bool verbose=false )
{
    std::cerr << std::endl;
    std::cerr << name << "perform conversion between RGB and YCrCb color spaces on image streams." << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat input.bin | image-color-calc <operation> [<options>] > output.bin " << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "  convert" << std::endl;
    std::cerr << "      conversion between RGB and YCrCb colorspaces" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options (operation-dependent)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "convert" << std::endl;
    std::cerr << "    --to=<colorspace>; default=YCrCb; direction of conversion, RGB or YCrCb (case-insensitive)" << std::endl;
    std::cerr << "    --from=<colorspace>; default=RGB; input colorspace, RGB or YCrCb (case-insensitive), opposite to direction of conversion" << std::endl;
    std::cerr << "    --header=[fields=<f>[;binary=<b>]]; input image is prepended with header that is omitted from conversion" << std::endl;
    std::cerr << "    --rows=<count>; default=1; the number of rows in the image" << std::endl;
    std::cerr << "    --cols,--columns=<count>; default=1; the number of columns in the image" << std::endl;
    std::cerr << "    --count,--size=<size>; default=1; the number of pixels in the image, rows times columns" << std::endl;
    std::cerr << std::endl;
    std::cerr << "general options" << std::endl;
    std::cerr << "    --binary=[<format>]: binary format of input stream" << std::endl;
    std::cerr << "    --format=[<format>]: format hint for ascii input (e.g., coefficients for convert depend on data type)" << std::endl;
    std::cerr << "    --fields=[<fields>]; default: operation-dependent" << std::endl;
    std::cerr << "    --flush; flush after every line or binary record" << std::endl;
    std::cerr << "    --input-fields; show default input fields (operation-dependent) and exit" << std::endl;
    std::cerr << "    --input-format; show default input format (operation-dependent) and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    convert" << std::endl;
    std::cerr << "        most simple, RGR to YCrCb (default), assume ub data" << std::endl;
    std::cerr << "            echo 1,2,3 | image-color-calc convert" << std::endl;
    std::cerr << "        input precision ambiguous, define explicitly; same default conversion applied" << std::endl;
    std::cerr << "            echo 1,0.2,0.3 | image-color-calc convert --format=3d" << std::endl;
    std::cerr << "        explicit conversion direction; '--to=RGB' would do the same" << std::endl;
    std::cerr << "            echo 1,0.2,0.3 | image-color-calc convert --from=YCrCb" << std::endl;
    std::cerr << "        handle binary, apply default conversion" << std::endl;
    std::cerr << "            echo 1,0.2,0.3 | csv-to-bin 3f | image-color-calc convert --binary=3f" << std::endl;
    std::cerr << "        input data have headers; skip header, apply default conversion to data" << std::endl;
    std::cerr << "            ... | image-color-calc convert --header=\"fields=id,t;binary=ui,t\" --rows=100 --cols=200" << std::endl;
    std::cerr << "        alternative form of the same" << std::endl;
    std::cerr << "            ... | image-color-calc convert --header=\"fields=id,t;binary=ui,t\" --count=$(( 100 * 200 ))" << std::endl;
    std::cerr << "        special fields in the header; rows, columns and format are taken from the header, cannot be given explicitly" << std::endl;
    std::cerr << "            ... | image-color-calc convert --header=\"fields=t,rows,cols,type;binary=t,3ui\"" << std::endl;
    std::cerr << "        using fields to select values to convert; RGB values 1,2,3 are converted to YCrCb" << std::endl;
    std::cerr << "            echo 'value',1,2,3,20170101T000000 | image-color-calc convert --fields=name,r,g,b,t" << std::endl;
    std::cerr << "        field names select conversion from YCrCb to RGB, precision explicitly hinted" << std::endl;
    std::cerr << "            echo 'value',1,2,3,20170101T000000 | image-color-calc convert --fields=name,y,cr,cb,t --format=,3ui," << std::endl;
    std::cerr << "        same example on binary data" << std::endl;
    std::cerr << "            echo 'value',1,2,3,20170101T000000 | csv-to-bin s[10],3ui,t | image-color-calc convert --fields=name,y,cr,cb,t --binary=s[10],3ui,t" << std::endl;
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
        using snark::cv_mat::serialization;
        verbose = options.exists("--verbose,-v");
        std::vector< std::string > ops = options.unnamed("-h,--help,-v,--verbose,--flush,--input-fields,--input-format,--output-fields,--output-format,--show-partial,--discard", "--fields,--binary,--input");
        if( ops.empty() ) { std::cerr << name << "please specify an operation." << std::endl; return 1;  }
        if( ops.size() > 1 ) { std::cerr << name << "please specify only one operation, got " << comma::join( ops, ' ' ) << std::endl; return 1; }
        std::string operation = ops.front();
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
                comma::csv::output_stream< serialization::header > ascii( std::cout, out );
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
                
                serialization::header header = serialization.get_header( &serialization.header_buffer()[0] );
                
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
                    serialization::header header = serialization.get_header( &serialization.header_buffer()[0] );
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
    catch( std::exception& ex ) { std::cerr << "image-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "image-calc: unknown exception" << std::endl; }
    return 1;
}

