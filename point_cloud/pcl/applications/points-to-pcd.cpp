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

/// @author mikkel kragh hansen

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <deque>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../../visiting/eigen.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/io/io.h>
#include <pcl/conversions.h>

struct input_point
{
    struct colour_t
    {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char a;
    };
    
    boost::posix_time::ptime timestamp;
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    colour_t colour;
    comma::uint32 intensity;
    comma::uint32 id;
    comma::uint32 block;

    input_point() : point( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ), block( 0 ) {}
};

struct input_fields
{
    bool timestamp;
    bool point;
    bool normal;
    bool color;
    bool alpha;
    bool intensity;
    bool id;
    bool block;

    input_fields() : timestamp(false), point(false), normal(false), color(false), alpha(false), intensity(false), id(false), block(false) {}
};

namespace comma { namespace visiting {

 template <> struct traits< input_point::colour_t >
 {
     template < typename K, typename V > static void visit( const K&, input_point::colour_t& p, V& v )
     {
         v.apply( "r", p.r );
         v.apply( "g", p.g );
         v.apply( "b", p.b );
         v.apply( "a", p.a );
     }

     template < typename K, typename V > static void visit( const K&, const input_point::colour_t& p, V& v )
     {
         v.apply( "r", p.r );
         v.apply( "g", p.g );
         v.apply( "b", p.b );
         v.apply( "a", p.a );
     }
 };
    
template <> struct traits< input_point >
{
    template < typename K, typename V > static void visit( const K&, input_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
        v.apply("colour",p.colour);
        v.apply( "intensity", p.intensity );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const input_point& p, V& v )
    {
    	v.apply( "t", p.timestamp );
		v.apply( "point", p.point );
		v.apply( "normal", p.normal );
		v.apply("colour",p.colour);
		v.apply( "intensity", p.intensity );
		v.apply( "id", p.id );
		v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {


static void usage( bool verbose )
{
    static const char * const usage_synopsis =
        "\n"
        "\nconvert points to .pcd files using Point Cloud Library (PCL):"
        "\n"
        "\nusage: points-to-pcd <filename> [<options>]"
        "\n"
        "\n<filename>: output pcd filename"
        "\n            if block field present, output filename prefix, e.g: points-to-pcd cloud --fields=point,block"
        "\n            will output blocks into files like: cloud.0.pcd, cloud.3.pcd, cloud.29.pcd, etc, where 0,3,29 are block ids"
        "\n";

    static const char * const usage_options =
        "\noutput options"
        "\n    --to-binary: save to binary pcd format instead of csv format; default ascii pcd"
        "\n";

    static const char * const usage_csv_options =
        "\n"
        "\nfields:"
        "\n    default: point/x,point/y,point/z"
        "\n    t: if present, use timestamp from the packet as pcd filename; if absent, use system time"
        "\n    point/x,point/y,point/z or x,y,z: coordinates (%d in binary)"
        "\n    normal/x,normal/y,normal/z: if present, add normal coordinates in PCL types PointXYZINormal (%d in binary)"
        "\n    colour/r,colour/g,colour/b or r,g,b: if present, add RGB colour in PCL types PointXYZRGB, PointXYZRGBL and PointXYZRGBNormal (0-255; %uc in binary)"
        "\n    colour/a or a: if present, add colour transparency in PCL type PointXYZRGBA (0-255, %uc in binary); default 255"
        "\n    intensity: if present, add intensity in PCL types PointXYZI and PointXYZINormal and PointXYZRGBNormal (%ui in binary)"
        "\n    id: if present, add id as label in PCL types PointXYZL and PointXYZRGBL (%ui in binary)"
        "\n    block: if present, accumulate and save each data block separately (%ui in binary)"
        "\n";

    static const char * const usage_examples =
        "\nbasics"
        "\n    convert a single point:"
        "\n        echo 1,2,3 | points-to-pcd"
        "\n"
        "\n    convert a single point with intensity and normal:"
        "\n        echo 1,2,3,10,0,0,1 | points-to-pcd --fields=point/x,point/y,point/z,intensity,normal/x,normal/y,normal/z"
        "\n"
        "\n    convert multiple points and split by block:"
        "\n        echo -e \"0,1,2,1\\n0,1,3,1\\n1,2,3,3\" | points-to-pcd --fields=point,block"
        "\n"
        "\n";

    std::cerr
        << usage_synopsis
        << usage_options
        << usage_csv_options;
    if( verbose ) { std::cerr << "\ncsv options\n" << comma::csv::options::usage(); }
    std::cerr
        << usage_examples
        << std::endl;
    exit( 0 );
}

static std::string filename;
input_fields fields_set;

template <typename PointT>
int csv2pcd(const std::deque< input_point > &cloudIn, bool to_binary)
{
    pcl::PointCloud< PointT > cloudOut;
    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    for (std::size_t i=0;i<cloudIn.size();i++)
    {
        PointT tmp;

        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "x", cloudIn[i].point(0)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "y", cloudIn[i].point(1)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "z", cloudIn[i].point(2)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "intensity", cloudIn[i].intensity));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "normal_x", cloudIn[i].normal(0)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "normal_y", cloudIn[i].normal(1)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "normal_z", cloudIn[i].normal(2)));
        comma::uint32 rgb = (static_cast<comma::uint32>(cloudIn[i].colour.r) << 16 | static_cast<comma::uint32>(cloudIn[i].colour.g) << 8 | static_cast<comma::uint32>(cloudIn[i].colour.b));
        comma::uint32 rgba = (static_cast<comma::uint32>(cloudIn[i].colour.a) << 24 | rgb);
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "rgb", *reinterpret_cast<float*>(&rgb)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,comma::uint32> (tmp, "rgba", rgba));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,comma::uint32> (tmp, "label", static_cast<comma::uint32>(cloudIn[i].id)));

        cloudOut.push_back(tmp);
    }

    std::string f = fields_set.block ? filename + "." + boost::lexical_cast< std::string >( cloudIn[0].block ) + ".pcd" : filename;
    if( to_binary ) { pcl::io::savePCDFileBinary( f, cloudOut ); }
    else { pcl::io::savePCDFileASCII( f, cloudOut ); }
    return 0;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        const std::vector< std::string >& unnamed = options.unnamed( "--verbose,-v,--to-binary", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-to-pcd: please specify output filename" << std::endl; return 1; }
        filename = unnamed[0];
        bool to_binary = options.exists( "--to-binary" );
        comma::csv::options csv( ac, av );
        csv.full_xpath = true;
        if( csv.fields.empty() ) { csv.fields = "point"; }
        std::vector< std::string > fields = comma::split( csv.fields, csv.delimiter );
        for( std::size_t i = 0; i < fields.size(); ++i )
        {
            if (fields[i] == "t" ) { fields_set.timestamp = true; }
            else if( fields[i] == "point" || fields[i] == "point/x" || fields[i] == "point/y" || fields[i] == "point/z" ) { fields_set.point = true; }
            else if( fields[i] == "x" || fields[i] == "y" || fields[i] == "z" ) { fields[i] = "point/" + fields[i] ; fields_set.point = true; }
            else if( fields[i] == "normal" || fields[i] == "normal/x" || fields[i] == "normal/y" || fields[i] == "normal/z" ) { fields_set.normal = true; }
            else if( fields[i] == "r" || fields[i] == "g" || fields[i] == "b" ) { fields[i] = "colour/" + fields[i]; fields_set.color = true; }
            else if( fields[i] == "colour/r" || fields[i] == "colour/g" || fields[i] == "colour/b" ) { fields_set.color = true; }
            else if( fields[i] == "colour" || fields[i] == "colour/a" ) { fields_set.color = true; fields_set.alpha = true; }
            else if( fields[i] == "a" ) { fields[i] = "colour/a"; fields_set.color = true; fields_set.alpha = true; }
            else if( fields[i] == "intensity" ) { fields_set.intensity = true; }
            else if( fields[i] == "id" ) { fields_set.id = true; }
            else if( fields[i] == "block" ) { fields_set.block = true; }
            else { std::cerr<< "expected field name, got: \""<< fields[i] << "\"" << std::endl; return 1; }
        }
        csv.fields = comma::join( fields, ',' );
        comma::csv::input_stream< input_point > is( std::cin, csv );
        #ifdef WIN32
        if( is.binary() ) { _setmode( _fileno( stdout ), _O_BINARY ); }
        #endif
        boost::optional< input_point > last;
        while( !std::cin.eof() && std::cin.good() )
        {
            std::deque< input_point > input_buffer;
            const input_point* p;
            for( bool block_done = false; !block_done; )
            {
                p = is.read();
                block_done = last && ( !p || p->block != last->block );
                if( last ) { input_buffer.push_back( *last ); }
                if( !p ) { break; }
                last = *p;
                if( csv.binary() ) { std::cout.write( is.binary().last(), csv.format().size() ); }
                else { std::cout << comma::join( is.ascii().last(), csv.delimiter ) << std::endl; }
            }

            if( input_buffer.empty() ) { break; }
            if (fields_set.point & fields_set.intensity & fields_set.normal)
                csv2pcd<pcl::PointXYZINormal>(input_buffer,to_binary);
            else if (fields_set.point & fields_set.intensity)
                csv2pcd<pcl::PointXYZI>(input_buffer,to_binary);
            else if (fields_set.point & fields_set.color & fields_set.normal)
                csv2pcd<pcl::PointXYZRGBNormal>(input_buffer,to_binary);
            else if (fields_set.point & fields_set.color & fields_set.alpha)
                csv2pcd<pcl::PointXYZRGBA>(input_buffer,to_binary);
            else if (fields_set.point & fields_set.color & fields_set.id)
                csv2pcd<pcl::PointXYZRGBL>(input_buffer,to_binary);
            else if (fields_set.point & fields_set.color)
                csv2pcd<pcl::PointXYZRGB>(input_buffer, to_binary);
            else if (fields_set.point & fields_set.id)
                csv2pcd<pcl::PointXYZL>(input_buffer, to_binary);
            else if (fields_set.point)
                csv2pcd<pcl::PointXYZ>(input_buffer, to_binary);

            if( !p ) { break; }
        }

        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-pcd: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-pcd: unknown exception" << std::endl; }
    return 1;
}
