// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

// #define PCL_NO_PRECOMPILE

#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "../../../visiting/eigen.h"
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

static void usage(bool detail)
{
    std::cerr<<"    points pcl calc operations" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <operation> [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "operations:"<< std::endl;
    std::cerr<< "    region-growing-segmentation"<< std::endl;
    std::cerr<< "    normal-estimator"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit"<<std::endl;
    std::cerr << "    --input-format: print input format and exit"<<std::endl;
    std::cerr << "    --output-fields: print output fields and exit"<<std::endl;
    std::cerr << "    --output-format: print output format and exit"<<std::endl;
    std::cerr<< "region-growing-segmentation options:"<< std::endl;
    std::cerr << "    --min-cluster-size: the minimum number of points that a cluster needs to contain in order to be considered valid; default: 50"<< std::endl;
    std::cerr << "    --max-cluster-size: the maximum number of points that a cluster needs to contain in order to be considered valid; default 1000000"<< std::endl;
    std::cerr << "    --number-of-neighbours: number of nearest neighbours used for KNN (k-nearest neighbours search); default: 30"<< std::endl;
    std::cerr << "    --smoothness-threshold: smoothness threshold used for testing the points; default: "<<3.0 / 180.0 * M_PI<< std::endl;
    std::cerr << "    --curvature-threshold: residual threshold used for testing the points; default: 1.0"<< std::endl;
    std::cerr << "    --discard: discard points with invalid cluster, when not specified all points will be written to output, invalid points with id=0"<< std::endl;
    std::cerr<< "normal-estimator options:"<< std::endl;
    std::cerr << "   --k,--k-neighbours: number of k nearest neighbors to use for the feature estimation; default: 50"<< std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << std::endl;
}

struct input_point
{
    std::uint32_t block;
    pcl::PointXYZ point;
    input_point() : block(0) { }
};

struct input_point_normal
{
    std::uint32_t block;
    pcl::PointXYZ point;
    pcl::Normal normal;
    input_point_normal() : block(0) { }
};

struct segmentation_output
{
    std::uint32_t id;
    segmentation_output(std::uint32_t id=0) : id(id) { }
};

namespace comma { namespace visiting {
    
template <> struct traits< pcl::PointXYZ >
{
    template< typename K, typename V > static void visit( const K& k, pcl::PointXYZ& p, V& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
    template< typename K, typename V > static void visit( const K& k, const pcl::PointXYZ& p, V& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
};

template <> struct traits< pcl::Normal >
{
    template< typename K, typename V > static void visit( const K& k, pcl::Normal& p, V& v )
    {
        v.apply( "x", p.normal_x );
        v.apply( "y", p.normal_y );
        v.apply( "z", p.normal_z );
        v.apply( "curvature", p.curvature );
    }
    template< typename K, typename V > static void visit( const K& k, const pcl::Normal& p, V& v )
    {
        v.apply( "x", p.normal_x );
        v.apply( "y", p.normal_y );
        v.apply( "z", p.normal_z );
        v.apply( "curvature", p.curvature );
    }
};


template <> struct traits< input_point >
{
    template< typename K, typename V > static void visit( const K& k, input_point& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
    }
    template< typename K, typename V > static void visit( const K& k, const input_point& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
    }
};

template <> struct traits< input_point_normal >
{
    template< typename K, typename V > static void visit( const K& k, input_point_normal& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
    }
    template< typename K, typename V > static void visit( const K& k, const input_point_normal& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
    }
};

template <> struct traits< segmentation_output >
{
    template< typename K, typename V > static void visit( const K& k, segmentation_output& p, V& v )
    {
        v.apply( "id", p.id);
    }
    template< typename K, typename V > static void visit( const K& k, const segmentation_output& p, V& v )
    {
        v.apply( "id", p.id);
    }
};

} } // namespace comma { namespace visiting {

class app_i
{
public:
    virtual ~app_i() { }
    virtual void print_input_fields()=0;
    virtual void print_input_format()=0;
    virtual void print_output_fields()=0;
    virtual void print_output_format()=0;
    virtual void run()=0;
};

template<typename T,typename S>
class app_t : public app_i
{
public:
    typedef T input_t;
    typedef S output_t;
protected:
    comma::csv::options csv;
    input_t default_input;
    std::vector<std::string> buffer;
    std::size_t reserve;
    
    virtual void clear()=0;
    virtual void push_back(const input_t& p)=0;
    virtual void process()=0;
    //process cloud
    virtual void write_output(comma::csv::output_stream<output_t>& os)=0;
public:
    app_t(const comma::command_line_options& options) : csv(options)
    {
        csv.full_xpath=true;
        reserve=options.value<std::size_t>("--reserve",10000);
    }
    void run()
    {
        comma::csv::input_stream<input_t> is( std::cin, csv, default_input);
        comma::csv::output_stream<output_t> os( std::cout, csv.binary(), true, csv.flush );
        std::uint32_t block=0;
        buffer.reserve(reserve);
        while( is.ready() || std::cin.good() )
        {
            const input_t* p=is.read();
            if ( (!p || block != p->block ) && buffer.size() )
            {
                process();
                write_output(os);
                //reset
                buffer.clear();
                clear();
            }
            if(!p){break;}
            block=p->block;
            buffer.push_back(is.last());
            push_back(*p);
        }
    }
    void print_input_fields()
    {
        std::cout<<comma::join( comma::csv::names<input_t>(true), ',' ) << std::endl;
    }
    void print_input_format()
    {
        std::cout<<comma::csv::format::value<input_t>() << std::endl;
    }
    void print_output_fields()
    {
        std::cout<<comma::join( comma::csv::names<output_t>(false), ',' ) << std::endl;
    }
    void print_output_format()
    {
        std::cout<<comma::csv::format::value<output_t>() << std::endl;
    }
};

struct normal_estimator : public app_t<input_point,pcl::Normal>
{
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    typename pcl::PointCloud<pcl::Normal> normals;
    int k;
    normal_estimator(const comma::command_line_options& options) : app_t(options),
        cloud(new pcl::PointCloud<pcl::PointXYZ>())
    {
        k=options.value<int>("--k,--k-neighbours",50);
    }
    void clear()
    {
        cloud->clear();
        normals.clear();
    }
    void push_back(const input_t& p)
    {
        cloud->push_back(p.point);
    }
    void process()
    {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(k);
        normal_estimator.compute(normals);
    }
    void write_output(comma::csv::output_stream<output_t>& os)
    {
        for(std::size_t i=0;i<buffer.size()&&i<normals.size();i++)
        {
            os.append(buffer[i],normals[i]);
        }
    }
};

struct region_growing_segmentation : public app_t<input_point_normal,segmentation_output>
{
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals;
    std::vector<output_t> outputs;

    int min_cluster_size;
    int max_cluster_size;
    unsigned number_of_neighbours;
    float smoothness_threshold;
    float curvature_threshold;
    bool discard;
    region_growing_segmentation(const comma::command_line_options& options) : app_t(options), 
        cloud(new pcl::PointCloud<pcl::PointXYZ>()), normals(new pcl::PointCloud <pcl::Normal>)
    {
        min_cluster_size=options.value<int>("--min-cluster-size",50);
        max_cluster_size=options.value<int>("--max-cluster-size",1000000);
        number_of_neighbours=options.value<unsigned>("--number-of-neighbours",30);
        smoothness_threshold=options.value<float>("--smoothness-threshold",3.0 / 180.0 * M_PI);
        curvature_threshold=options.value<float>("--curvature-threshold",1.0);
        discard=options.exists("--discard");
        cloud->points.reserve(reserve);
    }
    void clear()
    {
        cloud->clear();
        normals->clear();
        outputs.clear();
    }
    void push_back(const input_t& p)
    {
        cloud->push_back(p.point);
        normals->push_back(p.normal);
    }
    void process()
    {
        std::vector<pcl::PointIndices> clusters;
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//         comma::verbose<<"min,max,k "<<min_cluster_size<<" "<<max_cluster_size<<" "<<number_of_neighbours<<std::endl;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(number_of_neighbours);
        reg.setInputCloud(cloud);
        //reg.setIndices (indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothness_threshold);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.extract(clusters);
        outputs.resize(buffer.size(),segmentation_output(0));
        comma::verbose<<"clusters.size() "<<clusters.size()<<" clusters[0].indices.size() "<<clusters[0].indices.size()<<std::endl;
        for(std::uint32_t i=0;i<clusters.size();i++)
        {
            for(auto j : clusters[i].indices)
            {
                outputs[j].id=i+1;
            }
        }
    }
    void write_output(comma::csv::output_stream<output_t>& os)
    {
        for(std::size_t i=0;i<buffer.size()&&i<outputs.size();i++)
        {
            if( !discard || outputs[i].id)
            {
                os.append(buffer[i],outputs[i]);
            }
        }
    }
};

// todo:
// add bash completion
// mkdir points-pcl-calc
// mv app* to points-pcl-calc/app.h or common.h
// mv normal estimator to points-pcl-calc/feature.h
// mv region growing segmentation to points-pcl-calc/segmentation.h

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        const std::vector< std::string >& operations=options.unnamed( "--verbose,-v,--input-fields,--input-format,--output-fields,--output-format,--discard","-.*" );
        if(operations.size()!=1) { std::cerr<<comma::verbose.app_name()<<": expected one operation, got " << operations.size() << ": " << comma::join(operations,' ' ) << std::endl; return 1; }
        const std::string& operation=operations[0];
        std::unique_ptr<app_i> app;
        if(operation=="region-growing-segmentation")
        {
            app.reset(new region_growing_segmentation(options));
        }
        else if(operation=="normal-estimator")
        {
            app.reset(new normal_estimator(options));
        }
        else
        {
            std::cerr<<comma::verbose.app_name()<<": unrecognized operation "<<operation<<std::endl;
            return 1;
        }
        if(options.exists("--input-fields")) { app->print_input_fields(); return 0; }
        if(options.exists("--input-format")) { app->print_input_format(); return 0; }
        if(options.exists("--output-fields")) { app->print_output_fields(); return 0; }
        if(options.exists("--output-format")) { app->print_output_format(); return 0; }
        app->run();
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
