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
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include <snark/visiting/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/io/io.h>
#include <pcl/conversions.h>

struct input_point
{
    boost::posix_time::ptime timestamp;
    Eigen::Vector3d point;
    union
    {
        double normal[3];
        struct
        {
            double normal_x;
            double normal_y;
            double normal_z;
        };
    };
    union
    {
        float color;
        struct
        {
            unsigned char r;
            unsigned char g;
            unsigned char b;
            unsigned char a;
        };
    };
    comma::uint32 intensity;
    comma::uint32 id;
    comma::uint32 block;

    input_point() : block( 0 ) {}
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

template <> struct traits< input_point >
{
    template < typename K, typename V > static void visit( const K&, input_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "point", p.point );
        v.apply( "normal/x", p.normal_x ); // quick and dirty
        v.apply( "normal/y", p.normal_y ); // quick and dirty
        v.apply( "normal/z", p.normal_z ); // quick and dirty
        v.apply( "r", p.r );
        v.apply( "g", p.g );
        v.apply( "b", p.b );
        v.apply( "a", p.a );
        v.apply( "intensity", p.intensity );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const input_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "point", p.point );
        v.apply( "normal/x", p.normal_x ); // quick and dirty
        v.apply( "normal/y", p.normal_y ); // quick and dirty
        v.apply( "normal/z", p.normal_z ); // quick and dirty
        v.apply( "r", p.r );
        v.apply( "g", p.g );
        v.apply( "b", p.b );
        v.apply( "a", p.a );
        v.apply( "intensity", p.intensity );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {


static void usage()
{
    static const char * const usage_synopsis =
        "\n"
        "\nconvert points to .pcd files using Point Cloud Library (PCL):"
        "\n"
        "\nusage: points-to-pcd [<options>]"
        "\n";

    static const char * const usage_options =
        "\ninput data options"
        "\n    --to-binary: save to binary pcd format instead of csv format"
        "\n";

    static const char * const usage_csv_options =
        "\n"
        "\n    fields:"
        "\n        default: x,y,z"
        "\n        t: if present, use timestamp from the packet as pcd filename; if absent, use system time"
        "\n        x,y,z: coordinates (%d in binary)"
        "\n        normal/x,normal/y,normal/z: if present, add normal coordinates in PCL types PointXYZINormal (%d in binary)"
        "\n        r,g,b: if present, add RGB colour in PCL types PointXYZRGB, PointXYZRGBL and PointXYZRGBNormal (0-255; %uc in binary)"
        "\n        a: if present, add colour transparency in PCL type PointXYZRGBA (0-255, %uc in binary); default 255"
        "\n        intensity: if present, add intensity in PCL types PointXYZI and PointXYZINormal and PointXYZRGBNormal (%ui in binary)"
        "\n        id: if present, add id as label in PCL types PointXYZL and PointXYZRGBL (%ui in binary)"
        "\n        block: if present, accumulate and save each data block separately (%ui in binary)"
        "\n";

    static const char * const usage_examples =
        "\nbasics"
        "\n    convert a single point:"
        "\n        echo 1,2,3 | points-to-pcd"
        "\n"
        "\n    convert a single point with intensity and normal:"
        "\n        echo 1,2,3,10,0,0,1 | points-to-pcd --fields=x,y,z,intensity,normal/x,normal/y,normal/z"
        "\n"
        "\n    convert multiple points and split by block:"
        "\n        echo -e \"0,1,2,1\\n0,1,3,1\\n1,2,3,3\" | points-to-pcd --fields=x,y,z,block"
        "\n"
        "\n";

    std::cerr
        << usage_synopsis
        << usage_options
        << "\ncsv options\n"
        << comma::csv::options::usage()
        << usage_csv_options
        << usage_examples
        << std::endl;
    exit( 1 );
}

static comma::signal_flag is_shutdown;

template <typename PointT>
int csv2pcd(const std::deque< input_point > &cloudIn, pcl::PointCloud<PointT> &cloudOut, bool to_binary)
{
    // Set all relevant fields
    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    for (unsigned int i=0;i<cloudIn.size();i++)
    {
        PointT tmp;

        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "x", cloudIn[i].point(0)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "y", cloudIn[i].point(1)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "z", cloudIn[i].point(2)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "intensity", cloudIn[i].intensity));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "normal_x", cloudIn[i].normal_x));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "normal_y", cloudIn[i].normal_y));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "normal_z", cloudIn[i].normal_z));
        uint32_t rgb = (static_cast<uint32_t>(cloudIn[i].r) << 16 | static_cast<uint32_t>(cloudIn[i].g) << 8 | static_cast<uint32_t>(cloudIn[i].b));
        uint32_t rgba = (static_cast<uint32_t>(cloudIn[i].a) << 24 | rgb);
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,float> (tmp, "rgb", *reinterpret_cast<float*>(&rgb)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,uint32_t> (tmp, "rgba", rgba));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT,uint32_t> (tmp, "label", static_cast<uint32_t>(cloudIn[i].id)));

        cloudOut.push_back(tmp);
    }

    // Use timestamp as filename
    boost::posix_time::ptime timestamp;
    if (cloudIn[0].timestamp.is_not_a_date_time()) // If no timestamp is provided, use system time
        timestamp = boost::posix_time::microsec_clock::universal_time();
    else
        timestamp = cloudIn[0].timestamp;

    std::string timestamp_str = boost::posix_time::to_iso_string(timestamp);
    std::string filename = timestamp_str + boost::lexical_cast<std::string>(cloudIn[0].block) + ".pcd";

    // Save to file
    if (to_binary)
        pcl::io::savePCDFileBinary(filename, cloudOut);
    else
        pcl::io::savePCDFileASCII (filename, cloudOut);

    return 0;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }

        comma::csv::options input_options( ac, av );
        input_options.full_xpath = true;
        if( input_options.fields == "" ) { input_options.fields = "x,y,z"; }

        bool to_binary = false;
        if( options.exists( "--to-binary" ) ) { to_binary=true; }

        comma::csv::options output_options = input_options;

        comma::csv::input_stream< input_point > is( std::cin, input_options );
        comma::csv::output_stream< input_point > os( std::cout, output_options );

        std::vector< std::string > fields = comma::split( input_options.fields, input_options.delimiter );
//		input_options.fields = comma::join( fields, ',' );
        input_fields fields_set;
        for( std::size_t i = 0; i < fields.size(); ++i )
        {
            if (fields[i] == "t" ) { fields_set.timestamp = true; }
            if( fields[i] == "x" ) { fields_set.point = true; }
            else if( fields[i] == "y" ) { fields_set.point = true; }
            else if( fields[i] == "z" ) { fields_set.point = true; }
            if( fields[i] == "normal/x" ) { fields_set.normal = true; }
            else if( fields[i] == "normal/y" ) { fields_set.normal = true; }
            else if( fields[i] == "normal/z" ) { fields_set.normal = true; }
            if( fields[i] == "r" ) { fields_set.color = true; }
            else if( fields[i] == "g" ) { fields_set.color = true; }
            else if( fields[i] == "b" ) { fields_set.color = true; }
            if( fields[i] == "a" ) { fields_set.alpha = true; }
            if( fields[i] == "intensity" ) { fields_set.intensity = true; }
            if( fields[i] == "id" ) { fields_set.id = true; }
            if( fields[i] == "block" ) { fields_set.block = true; }
        }

        boost::optional< input_point > last;
        while ( !std::cin.eof() && std::cin.good() )
        {
            std::deque< input_point > input_buffer;
            const input_point* p;
            for( bool block_done = false; !block_done;  )
            {
                p = is.read();
                block_done = last && ( !p || p->block != last->block );

                if( last )
                {
                    input_buffer.push_back( *last );
                    input_point out;
                    out = *last;

                    if( input_options.binary() ) { os.write( out, is.binary().last()); }
                    else { os.write( out, is.ascii().last()); }
                }

                if (p) { last = *p; }
            }

            // Cast to relevant PCL point cloud type
            if (fields_set.point & fields_set.intensity & fields_set.normal)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZINormal> (),to_binary);
            else if (fields_set.point & fields_set.intensity)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZI> (),to_binary);
            else if (fields_set.point & fields_set.color & fields_set.normal)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZRGBNormal> (),to_binary);
            else if (fields_set.point & fields_set.color & fields_set.alpha)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZRGBA> (),to_binary);
            else if (fields_set.point & fields_set.color & fields_set.id)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZRGBL> (),to_binary);
            else if (fields_set.point & fields_set.color)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZRGB> (),to_binary);
            else if (fields_set.point & fields_set.id)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZL> (),to_binary);
            else if (fields_set.point)
                csv2pcd(input_buffer,* new pcl::PointCloud<pcl::PointXYZ> (),to_binary);

            if( !p ) { break; }
        }

        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-pcd: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-pcd: unknown exception" << std::endl; }
    return 1;
}
