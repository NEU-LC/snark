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

struct input_t
{
	std::string filename;
};

struct output_point
{
	boost::posix_time::ptime timestamp;
	Eigen::Vector3d point;
	Eigen::Vector3d normal;
	struct colour_
	{
		unsigned char r;
		unsigned char g;
		unsigned char b;
		unsigned char a;
	} colour;
	comma::uint32 intensity;
	comma::uint32 id;
	comma::uint32 block;

	output_point() : point( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ), block( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
	template < typename K, typename V > static void visit( const K&, input_t& p, V& v )
	{
		v.apply( "filename", p.filename );
	}

	template < typename K, typename V > static void visit( const K&, const input_t& p, V& v )
	{
		v.apply( "filename", p.filename );
	}
};

template <> struct traits< output_point::colour_ >
{
    template < typename K, typename V > static void visit( const K&, output_point::colour_& p, V& v )
    {
        v.apply( "r", p.r );
        v.apply( "g", p.g );
        v.apply( "b", p.b );
        v.apply( "a", p.a );
    }

    template < typename K, typename V > static void visit( const K&, const output_point::colour_& p, V& v )
    {
        v.apply( "r", p.r );
        v.apply( "g", p.g );
        v.apply( "b", p.b );
        v.apply( "a", p.a );
    }
};

template <> struct traits< output_point >
{
	template < typename K, typename V > static void visit( const K&, output_point& p, V& v )
	{
		v.apply( "t", p.timestamp );
		v.apply( "point", p.point );
		v.apply( "normal", p.normal );
		v.apply( "colour", p.colour);
		v.apply( "intensity", p.intensity );
		v.apply( "id", p.id );
		v.apply( "block", p.block );
	}

	template < typename K, typename V > static void visit( const K&, const output_point& p, V& v )
	{
		v.apply( "t", p.timestamp );
		v.apply( "point", p.point );
		v.apply( "normal", p.normal );
		v.apply( "colour", p.colour);
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
			"\nread .pcd files using Point Cloud Library (PCL):"
			"\n"
			"\nusage: points-from-pcd [<options>]"
			"\n";

	static const char * const usage_options =
			"\ninput options"
			"\n    --output-fields: print available output fields in the pcd file"
			"\n"
			"\noutput options"
			"\n    --binary, -b: if present, output in binary equivalent of csv"
			"\n    --fields <fields>: e.g. point/x,point/y,point/z"
			"\n";

	static const char * const usage_csv_options =
			"\n"
			"\n    fields:"
			"\n        default: outputs all available fields in the pcd file"
			"\n        t: if applicable, output timestamp from filename for all points; if not, use system time"
			"\n        point/x,point/y,point/z: coordinates (%d in binary)"
			"\n        normal/x,normal/y,normal/z: normal coordinates (%d in binary)"
			"\n        colour/r,colour/g,colour/b: RGB colours (0-255; %uc in binary)"
			"\n        colour/a: colour transparency (0-255, %uc in binary)"
			"\n        intensity: point intensity (%ui in binary)"
			"\n        id: id/label (%ui in binary)"
			"\n        block: increasing number for each processed pcd file (%ui in binary)"
			"\n";

	static const char * const usage_examples =
			"\nbasics"
			"\n    read a pcd file:"
			"\n        echo \"20160118T222144.9507515.pcd\" | points-from-pcd"
			"\n"
			"\n    read specific fields from a pcd file:"
			"\n        echo \"20160118T222144.9507515.pcd\" | points-from-pcd --fields=t,point,colour/r,colour/g,colour/b"
			"\n"
			"\n    print available fields from a pcd file:"
			"\n        echo \"20160118T222144.9507515.pcd\" | points-from-pcd --output-fields"
			"\n"
			"\n    read a pcd file and output binary:"
			"\n        echo \"20160118T222144.9507515.pcd\" | points-from-pcd --fields=t,point/x,point/y,point/z --binary"
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

int main( int ac, char** av )
{
	try
	{
		comma::command_line_options options( ac, av );
		if( options.exists( "--help,-h" ) ) { usage(); }
		comma::csv::options input_options( ac, av );
		input_options.full_xpath = true;

		comma::csv::options output_options( input_options );

		input_options.fields="filename";
		comma::csv::input_stream< input_t > is( std::cin, input_options );

		comma::uint32 n = 1;
		while ( !is_shutdown && !std::cin.eof() && std::cin.good() )
		{
			const input_t* p = is.read();
			if( !p ) { break; }

			// Load point cloud
			pcl::PCLPointCloud2::Ptr cloud;
			cloud.reset (new pcl::PCLPointCloud2);
			std::string filename = p->filename;
			if (pcl::io::loadPCDFile(filename, *cloud) == -1) //* load the file
			{
				std::cerr << "points-from-pcd: error loading file: " << filename << std::endl;
				return 1;
			}

			// Check available fields in pcd file
			bool has_xyz = false;
			bool has_normal = false;
			bool has_intensity = false;
			bool has_label = false;
			bool has_rgb = false;
			bool has_rgba = false;
			for (size_t i=0;i<cloud->fields.size();i++)
			{
				if ((cloud->fields[i].name == "x") || (cloud->fields[i].name == "y") || (cloud->fields[i].name == "z"))
					has_xyz = true;
				if ((cloud->fields[i].name == "normal_x") || (cloud->fields[i].name == "normal_y") || (cloud->fields[i].name == "normal_z"))
					has_normal = true;
				if (cloud->fields[i].name == "intensity")
					has_intensity = true;
				if (cloud->fields[i].name == "label")
					has_label = true;
				if (cloud->fields[i].name == "rgb")
					has_rgb = true;
				if (cloud->fields[i].name == "rgba")
					has_rgba = true;
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::Normal>::Ptr cloudNormal(new pcl::PointCloud<pcl::Normal> ());
			pcl::PointCloud<pcl::RGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::RGB> ());
			pcl::PointCloud<pcl::Intensity>::Ptr cloudI(new pcl::PointCloud<pcl::Intensity> ());
			pcl::PointCloud<pcl::Label>::Ptr cloudL(new pcl::PointCloud<pcl::Label> ());

			if (has_xyz) { pcl::fromPCLPointCloud2 (*cloud, *cloudXYZ); }
			if (has_normal) { pcl::fromPCLPointCloud2 (*cloud, *cloudNormal); }
			if (has_intensity) { pcl::fromPCLPointCloud2 (*cloud, *cloudI); }
			if (has_label) { pcl::fromPCLPointCloud2 (*cloud, *cloudL); }
			if (has_rgb || has_rgba) { pcl::fromPCLPointCloud2 (*cloud, *cloudRGB); }

			// Available fields in pcd file
			comma::csv::options available;
			available.full_xpath = true;
			available.fields = "t";
			if (has_xyz) { available.fields+=",point/x,point/y,point/z"; }
			if (has_normal) { available.fields+=",normal/x,normal/y,normal/z"; }
			if (has_intensity) { available.fields+=",intensity"; }
			if (has_label) { available.fields+=",id"; }
			if (has_rgb || has_rgba) { available.fields+=",colour/r,colour/g,colour/b"; }
			if (has_rgba) { available.fields+=",colour/a"; }
			available.fields+=",block";

			std::vector< std::string > fields = comma::split( output_options.fields, input_options.delimiter );
			if (output_options.fields == "")
			{
				// If no fields are specified, output all available fields from pcd file
				output_options.fields = available.fields;
			}
			else
			{
				// If fields are specified, check if all of these are available in the pcd file
				for (size_t i=0;i<fields.size();i++)
				{
					if ((!available.has_field(fields[i])) && (!available.has_field(fields[i]+"/x")) && (!available.has_field(fields[i]+"/r"))) // dirty hack for handling full xpaths point/x,y,z, normal/x,y,z and colour/r,g,b (has_field doesn't seem to support xpaths?)
					{
						std::cerr<<"field not available: "<<fields[i]<<std::endl;
						return 1;
					}
				}
			}

			if( options.exists( "--output-fields" ) )
			{
				std::cout<<output_options.fields<<std::endl;
				return 0;
			}

			if( options.exists( "--binary,-b" ) )
			{
				comma::csv::format format;

				for (size_t i=0;i<fields.size();i++)
				{
					if (fields[i]=="t") { format+="t"; }
					else if (fields[i]=="t") { format+="t"; }
					else if (fields[i]=="point/x") { format+="d"; }
					else if (fields[i]=="point/y") { format+="d"; }
					else if (fields[i]=="point/z") { format+="d"; }
					else if (fields[i]=="normal/x") { format+="d"; }
					else if (fields[i]=="normal/y") { format+="d"; }
					else if (fields[i]=="normal/z") { format+="d"; }
					else if (fields[i]=="intensity") { format+="ui"; }
					else if (fields[i]=="id") { format+="ui"; }
					else if (fields[i]=="colour/r") { format+="ub"; }
					else if (fields[i]=="colour/g") { format+="ub"; }
					else if (fields[i]=="colour/b") { format+="ub"; }
					else if (fields[i]=="colour/a") { format+="ub"; }
					else if (fields[i]=="block") { format+="ui"; }
				}
				output_options.format( format );
			}

			comma::csv::output_stream< output_point > os( std::cout, output_options );

			for (unsigned int i=0;i<cloud->width;i++)
			{
				output_point out;

				// Use timestamp as filename, if applicable
				try {
					out.timestamp = boost::posix_time::from_iso_string(filename);
				} catch ( ... ) {
					out.timestamp = boost::posix_time::microsec_clock::universal_time();
				}

				out.block = static_cast<comma::uint32>(n);

				if (has_xyz)
				{
					out.point(0) = cloudXYZ->points[i].x;
					out.point(1) = cloudXYZ->points[i].y;
					out.point(2) = cloudXYZ->points[i].z;
				}
				if (has_normal)
				{
					out.normal(0) = cloudNormal->points[i].normal_x;
					out.normal(1) = cloudNormal->points[i].normal_y;
					out.normal(2) = cloudNormal->points[i].normal_z;
				}
				if (has_intensity)
					out.intensity = static_cast<uint32_t>(cloudI->points[i].intensity);
				if (has_label)
					out.id = static_cast<uint32_t>(cloudL->points[i].label);
				if (has_rgb | has_rgba)
				{
					out.colour.r = static_cast<unsigned char>(cloudRGB->points[i].r);
					out.colour.g = static_cast<unsigned char>(cloudRGB->points[i].g);
					out.colour.b = static_cast<unsigned char>(cloudRGB->points[i].b);
				}
				if (has_rgba)
					out.colour.a = static_cast<unsigned char>(cloudRGB->points[i].a);

				os.write( out );
			}
			n++;
		}

		return 0;
	}
	catch( std::exception& ex ) { std::cerr << "points-from-pcd: " << ex.what() << std::endl; }
	catch( ... ) { std::cerr << "points-from-pcd: unknown exception" << std::endl; }
	return 1;
}
