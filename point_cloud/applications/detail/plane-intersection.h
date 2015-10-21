#pragma once
#include <iostream>
#include <comma/csv/stream.h>
#include <comma/base/exception.h>
#include <comma/application/verbose.h>
#include <snark/visiting/eigen.h>

struct plane_intersection
{
    typedef Eigen::Vector3d output_t;
    typedef Eigen::Vector3d vector;
    static void usage();
    //updates csv with default fields if --plane option specified
    static void process(const comma::command_line_options& options, comma::csv::options csv);
//     struct input_t
//     {
//         Eigen::Vector3d point;
//         Eigen::Vector3d normal;
//         input1_t():point( Eigen::Vector3d::Zero() ),normal(Eigen::Vector3d::Zero()){}
//         input1_t( const Eigen::Vector3d&  default_normal):point( Eigen::Vector3d::Zero() ),normal(default_normal){}
//     };
    static inline vector zero() { return Eigen::Vector3d::Zero(); }
    struct line_t
    {
        std::pair<vector, vector> points;
        line_t() : points(zero(), zero()) { }
        vector direction() const { return points.second - points.first; }
    };
    struct plane_t
    {
        vector point;
        vector normal;
        plane_t() : point(zero()), normal(zero()) { }
        plane_t(const plane_t& rhs) : point(rhs.point), normal(rhs.normal) { }
    };
    struct input_t
    {
        line_t line;
        plane_t plane;
        input_t() { }
        input_t(const plane_t& p):plane(p) { }
    };
    static void run( const comma::csv::options& csv_opt, boost::optional<std::string> plane_option, bool discard_collinear);
};

namespace comma { namespace visiting {

template <> struct traits< plane_intersection::line_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::line_t& t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::line_t& t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
};

template <> struct traits< plane_intersection::plane_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::plane_t & t, V& v )
    {
        v.apply( "point", t.point);
        v.apply( "normal", t.normal);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::plane_t & t, V& v )
    {
        v.apply( "point", t.point);
        v.apply( "normal", t.normal);
    }
};

template <> struct traits< plane_intersection::input_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::input_t& t, V& v )
    {
        traits<plane_intersection::line_t>::visit(k, t.line, v);
        v.apply( "plane", t.plane);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::input_t& t, V& v )
    {
        traits<plane_intersection::line_t>::visit(k, t.line, v);
        v.apply( "plane", t.plane);
    }
};


} } // namespace comma { namespace visiting {
  
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
std::string to_string(const Eigen::Vector3d& v)
{
    std::stringstream ss;
    ss<<"("<<v.x()<<", "<<v.y()<<", "<<v.z()<<")";
    return ss.str();
}
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::line_t& line)
{
    stream << "line_t{"<<to_string(line.points.first)<<"; "<<to_string(line.points.second)<<"}";
    return stream;
}
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::plane_t& plane)
{
    stream << "plane_t{"<<to_string(plane.point)<<"; "<<to_string(plane.normal)<<"}";
    return stream;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++

void plane_intersection::usage()
{
    std::cerr << "    plane-intersection: read points and plane from stdin, output intersection of the ray with the plane" << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names<plane_intersection::input_t>(true), ',' ) << std::endl;
    std::cerr << "            first and second are two points on the line; the plane passes through the plane/point and is perpendicular to the direction vector plane/normal" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --plane=<" << comma::join( comma::csv::names<plane_intersection::plane_t>(true), ',') <<  ">: default values for plane" << std::endl;
    std::cerr << "                if --plane specified, default fields for input are: " << comma::join( comma::csv::names<plane_intersection::line_t>(true), ',') <<  std::endl;
    std::cerr << "            --input-fields: print default input field names" << std::endl;
    std::cerr << "            --output-fields: print output field names" << std::endl;
    std::cerr << "            --output-format: print output fields format" << std::endl;
    std::cerr << "            --discard-collinear: don't output records when line is in parallel with plane (when not sepcified it outputs <inf,inf,inf>)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        example:" << std::endl;
    std::cerr << "            to find intersection of a line going from 0,0,0 to 1,1,1 and a plane at 0,0,4 perpendicular to z axis:" << std::endl;
    std::cerr << "                echo 1,1,1,4 | points-calc plane-intersection --plane=0,0,0,0,0,1 --fields=second,plane/point/z" << std::endl;
    std::cerr << "            outputs:  1,1,1,4,4,4,4" << std::endl;
    std::cerr << std::endl;
}

void plane_intersection::process(const comma::command_line_options& options, comma::csv::options csv)
{
    if(options.exists("--input-fields")) { std::cout<<comma::join( comma::csv::names<plane_intersection::input_t>(true), ',' ) << std::endl; return; }
    if(options.exists("--output-fields")){std::cout<<comma::join( comma::csv::names<plane_intersection::output_t>(false), ',' ) << std::endl; return; }
    if(options.exists("--output-format")){std::cout<<comma::csv::format::value<plane_intersection::output_t>() << std::endl; return; }
    boost::optional<std::string> plane_option=options.optional<std::string>("--plane");
    if(plane_option && csv.fields.empty()) { csv.fields=comma::join( comma::csv::names<plane_intersection::line_t>(true), ','); }
    run(csv, plane_option, options.exists( "--discard-collinear" ));
}

void plane_intersection::run( const comma::csv::options& csv_opt, boost::optional<std::string> plane_option, bool discard_collinear)
{
    plane_t plane = plane_option ? comma::csv::ascii<plane_t>().get(*plane_option) : plane_t();
    comma::csv::input_stream<plane_intersection::input_t > is( std::cin, csv_opt, plane_intersection::input_t (plane));
    comma::csv::output_stream<plane_intersection::output_t> os( std::cout, csv_opt.binary() );
    comma::csv::tied<plane_intersection::input_t ,plane_intersection::output_t> tied(is,os);
    static const Eigen::Vector3d infinity( std::numeric_limits< double >::infinity(), std::numeric_limits< double >::infinity(), std::numeric_limits< double >::infinity() );
    while(is.ready() || std::cin.good())
    {
        const plane_intersection::input_t* rec=is.read();
        if(!rec){break;}
        comma::verbose<<"line: "<< rec->line << ";" <<" plane: "<< rec->plane <<std::endl;
        double u=(rec->plane.point - rec->line.points.first).dot(rec->plane.normal);
        double d=rec->line.direction().dot(rec->plane.normal);
        comma::verbose<<"u "<<u<<", d "<<d<<std::endl;
        //if both u and d are 0 then line is on the plane and we just output first point
        if ( comma::math::equal(u,0)) { tied.append(rec->line.points.first); continue; }
        if( comma::math::equal( d, 0 ) ) { if( !discard_collinear ) { tied.append( infinity ); } continue; }
        tied.append( (u/d) * rec->line.direction() + rec->line.points.first );
    }
}
