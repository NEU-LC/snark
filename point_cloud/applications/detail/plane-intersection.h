#pragma once
#include <iostream>
#include <comma/csv/stream.h>
#include <comma/base/exception.h>
#include <snark/visiting/eigen.h>

extern comma::csv::options csv;
extern bool verbose;
extern std::string app_name;

template<class I >
class point;
struct plane_intersection
{
    static void usage();
    struct fields_t
    {
        std::vector<std::string> vfields;
        bool contains(const std::string& name) const;
        bool start_with(const std::string& name) const;
        fields_t(const std::string& s) { vfields=comma::split(s, ","); }
    };
    template<typename L>
    static void step2(const fields_t& fields, const boost::optional<std::string>& plane_option);
    static void process(const comma::command_line_options& options);
    struct input1_t
    {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        input1_t():point( Eigen::Vector3d::Zero() ),normal(Eigen::Vector3d::Zero()){}
        input1_t( const Eigen::Vector3d&  default_normal):point( Eigen::Vector3d::Zero() ),normal(default_normal){}
    };
    typedef Eigen::Vector3d output_t;
    static void run( const comma::csv::options& csv_opt, const Eigen::Vector3d& normal);
    typedef Eigen::Vector3d vector;
    static inline vector zero() { return Eigen::Vector3d::Zero(); }
    struct line_t
    {
        // y = direction * x + point0
        virtual vector point0() const =0;
        virtual vector direction() const =0;
    };
    struct plane_t
    {
        // (x-point0) . normal = 0
        virtual vector point0() const =0;
        virtual vector normal() const =0;
    };
    template<typename L, typename P>
    struct input_t
    {
        L line_;
        P plane_;
        input_t() { }
        input_t(const P& p):plane_(p) { }
        const line_t& line() const { return (const line_t&)line_; }
        const plane_t& plane() const { return (const plane_t&)plane_; }
        static void usage();
    };
    struct line0_t : public line_t
    {
        vector dir;
        virtual vector point0() const { return zero(); }
        virtual vector direction() const { return dir; }
        line0_t() : dir(zero()) { }
        static std::string help() { return "x,y,z  is direction of the line, line passes through (0, 0, 0) "; }
    };
    struct line1_t : public line_t
    {
        std::pair<vector, vector> points;
        virtual vector point0() const { return points.first; }
        virtual vector direction() const { return points.second - points.first; }
        line1_t() : points(zero(), zero()) { }
        static std::string help() { return "first and second points are on the line (direction is second - first)"; }
    };
    struct line2_t : public line_t
    {
        vector point;
        vector dir;
        virtual vector point0() const { return point; }
        virtual vector direction() const { return dir; }
        line2_t() : point(zero()), dir(zero()) { }
        static std::string help() { return "x,y,z is a point on the line, dir is direction of the line"; }
    };
    struct plane0_t : public plane_t
    {
        vector norm;
        virtual vector point0() const { return zero(); }
        virtual vector normal() const { return norm;}
        plane0_t() : norm(zero()) { }
        //plane0_t(const plane0_t& rhs) : norm(rhs.norm) { }
        static std::string help() { return "x,y,z is plane's normal vector (perpendicular to plane), plane passes through x,y,z as well"; }
    };
    struct plane1_t : public plane_t
    {
        vector point;
        vector norm;
        virtual vector point0() const { return point; }
        virtual vector normal() const { return norm;}
        plane1_t() : point(zero()), norm(zero()) { }
        plane1_t(const plane1_t& rhs) : point(rhs.point), norm(rhs.norm) { }
        static std::string help() { return "x,y,z is a point on the plane, normal is plane's normal vector"; }
    };
    struct plane2_t : public plane_t
    {
        std::pair<vector, vector> points;
        virtual vector point0() const { return points.first; }
        virtual vector normal() const { return points.second - points.first;}
        plane2_t() : points(zero(), zero()) { }
        static std::string help() { return "first point is on the plane, plane's normal is calculated as (second - first)"; }
    };
    struct plane3_t : public plane_t
    {
        vector points[3];
        virtual vector point0() const { return points[0]; }
        virtual vector normal() const { return (points[2] - points[0]).cross(points[1] - points[0]) ;}
        plane3_t() { for (std::size_t i=0;i<sizeof(points)/sizeof(points[0]);i++) { points[i]=zero(); } }
        static std::string help() { return "the three points are on the plane (normal is calculated implicitly)"; }
    };
    template<typename L, typename P>
    static void run( const comma::csv::options& csv_opt, boost::optional<std::string> plane_option);
};

namespace comma { namespace visiting {

template <> struct traits< plane_intersection::input1_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::input1_t& t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "normal", t.normal);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::input1_t& t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "normal", t.normal);
    }
};

template <> struct traits< plane_intersection::line0_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::line0_t& t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.dir, v);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::line0_t& t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.dir, v);
    }
};
template <> struct traits< plane_intersection::line1_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::line1_t& t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::line1_t& t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
};
template <> struct traits< plane_intersection::line2_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::line2_t & t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "dir", t.dir);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::line2_t & t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "dir", t.dir);
    }
};
template <> struct traits< plane_intersection::plane0_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::plane0_t & t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.norm, v);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::plane0_t & t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.norm, v);
    }
};
template <> struct traits< plane_intersection::plane1_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::plane1_t & t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "normal", t.norm);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::plane1_t & t, V& v )
    {
        traits<Eigen::Vector3d>::visit(k, t.point, v);
        v.apply( "normal", t.norm);
    }
};
template <> struct traits< plane_intersection::plane2_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::plane2_t & t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::plane2_t & t, V& v )
    {
        traits<std::pair<Eigen::Vector3d, Eigen::Vector3d> >::visit(k, t.points, v);
    }
};
template <> struct traits< plane_intersection::plane3_t >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::plane3_t & t, V& v )
    {
        v.apply( "point0", t.points[0]);
        v.apply( "point1", t.points[1]);
        v.apply( "point2", t.points[2]);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::plane3_t & t, V& v )
    {
        v.apply( "point0", t.points[0]);
        v.apply( "point1", t.points[1]);
        v.apply( "point2", t.points[2]);
    }
};

template <typename L, typename P> struct traits< plane_intersection::input_t<L,P> >
{
    template< typename K, typename V > static void visit( const K& k, plane_intersection::input_t<L,P>& t, V& v )
    {
        v.apply( "line", t.line_);
        v.apply( "plane", t.plane_);
    }
    template< typename K, typename V > static void visit( const K& k, const plane_intersection::input_t<L,P>& t, V& v )
    {
        v.apply( "line", t.line_);
        v.apply( "plane", t.plane_);
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
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::line1_t& line)
{
    stream << "line1_t{"<<to_string(line.points.first)<<"; "<<to_string(line.points.second)<<"}";
    return stream;
}
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::line2_t& line)
{
    stream << "line2_t{"<<to_string(line.point)<<"; "<<to_string(line.dir)<<"}";
    return stream;
}
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::plane1_t& plane)
{
    stream << "plane1_t{"<<to_string(plane.point)<<"; "<<to_string(plane.norm)<<"}";
    return stream;
}
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::plane2_t& plane)
{
    stream << "plane2_t{"<<to_string(plane.points.first)<<"; "<<to_string(plane.points.second)<<"}";
    return stream;
}
static std::ostream& operator <<(std::ostream& stream, const plane_intersection::plane3_t& plane)
{
    stream << "plane3_t{"<<to_string(plane.points[0])<<"; "<<to_string(plane.points[1])<<", "<<plane.points[2]<<"}";
    return stream;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
template < typename V >
inline std::string join_embraced( const std::string& left, const V& v, const std::string& right , const std::string comma=",")
{ 
    std::ostringstream oss;
    for( typename V::const_iterator it = v.begin(); it != v.end(); it++)
    {
        if(it != v.begin()) { oss << comma; }
        oss << left << *it << right;
    }
    return oss.str();
}

template <typename L, typename P>
void plane_intersection::input_t<L,P>::usage()
{
    std::cerr << "            input fields: combiation of line fields and plane fields" << comma::join( comma::csv::names<plane_intersection::input_t<L, P> >(true), ',' ) << std::endl;
    std::cerr << "                line: "<< L::help()  << std::endl;
    std::cerr << "                plane: " << P::help() << std::endl;
    std::cerr << "            options: --plane=" << join_embraced( "<", comma::csv::names<P>(true), ">") <<  ": default values for plane" << std::endl;
    
}
void plane_intersection::usage()
{
    std::cerr << "    plane-intersection: read points and planes from stdin, output intersection of the ray with the plane" << std::endl;
    std::cerr << "        original form: " << std::endl;
    std::cerr << "            input fields: " << comma::join( comma::csv::names<plane_intersection::input1_t>(true), ',' ) << std::endl;
    std::cerr << "                x,y,z is direction of the line passing through (0,0,0); the plane passes through the normal/x,y,z and is perpendicular to it" << std::endl;
    std::cerr << "            options: --normal=<x>,<y>,<z>: default normal; uses this value when normal is empty or any of its fields are omitted" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        default extended form, when --extended is specified without field names:" << std::endl;
    plane_intersection::input_t<plane_intersection::line2_t, plane_intersection::plane1_t>::usage();
    std::cerr << std::endl;
    std::cerr << "        other extended forms: if the field names match, one of following forms will be used" << std::endl;
    std::cerr << "        input fields: combination of line fields and plane fields" << std::endl;
    std::cerr << "        options: --plane=<values matching plane field names>: default values for plane" << std::endl;
    std::cerr << "        line can be defined as: "<< std::endl;
    std::cerr << "            line fields: "<<comma::join( comma::csv::names<plane_intersection::line1_t>(true), ',' ) << std::endl;
    std::cerr << "            description: "<<plane_intersection::line1_t::help() << std::endl;
    std::cerr << std::endl;
    std::cerr << "            line fields: "<<comma::join( comma::csv::names<plane_intersection::line2_t>(true), ',' ) << std::endl;
    std::cerr << "            description: "<<plane_intersection::line2_t::help() << std::endl;
    std::cerr << std::endl;
    std::cerr << "        plane can be defined as: "<< std::endl;
    std::cerr << "            plane fields: "<<comma::join( comma::csv::names<plane_intersection::plane1_t>(true), ',' ) << std::endl;
    std::cerr << "            description: "<<plane_intersection::plane1_t::help() << std::endl;
    std::cerr << std::endl;
    std::cerr << "            plane fields: "<<comma::join( comma::csv::names<plane_intersection::plane2_t>(true), ',' ) << std::endl;
    std::cerr << "            description: "<<plane_intersection::plane2_t::help() << std::endl;
    std::cerr << std::endl;
    std::cerr << "            plane fields: "<<comma::join( comma::csv::names<plane_intersection::plane3_t>(true), ',' ) << std::endl;
    std::cerr << "            description: "<<plane_intersection::plane3_t::help() << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --input-fields: print default input field names (use --extended for default extended form)" << std::endl;
    std::cerr << "            --output-fields: print output field names" << std::endl;
    std::cerr << "            --output-format: print output fields format" << std::endl;
    std::cerr << "            --extended: use extended form for input fields" << std::endl;
    std::cerr << std::endl;
}

bool plane_intersection::fields_t::contains(const std::string& name) const
{
    std::string match[4] = { name, name + "/x", name + "/y", name + "/z"};
    for(std::vector<std::string>::const_iterator it = vfields.begin(); it != vfields.end(); it++)
    {
        for(std::size_t j=0;j<sizeof(match)/sizeof(match[0]);j++)
            if(*it == match[j])
                return true;
    }
    return false;
}
bool plane_intersection::fields_t::start_with(const std::string& name) const
{
    for(std::vector<std::string>::const_iterator it = vfields.begin(); it != vfields.end(); it++)
    {
        if(it->find(name)==0)
            return true;
    }
    return false;
}
template<typename L>
void plane_intersection::step2(const plane_intersection::fields_t& fields, const boost::optional<std::string>& plane_option)
{
    if(fields.contains("plane") || fields.contains("plane/normal")) //default
        run<L, plane_intersection::plane1_t >(csv, plane_option);
    else if(fields.contains("plane/first") || fields.contains("plane/second"))
        run<L, plane_intersection::plane2_t >(csv, plane_option);
    else if(fields.contains("plane/point0") || fields.contains("plane/point1") || fields.contains("plane/point2"))
        run<L, plane_intersection::plane3_t >(csv, plane_option);
    else if(!fields.start_with("plane"))
        run<L, plane_intersection::plane1_t >(csv, plane_option);   //use default option
    else
        COMMA_THROW( comma::exception, app_name << ": plane-intersection: could not match field names for plane. field names: " << comma::join(fields.vfields, ',') );
}

void plane_intersection::process(const comma::command_line_options& options)
{
    if(options.exists("--input-fields"))
    {
        if(options.exists("--extended")) { std::cout<<comma::join( comma::csv::names<plane_intersection::input_t<plane_intersection::line2_t, plane_intersection::plane1_t> >(true), ',' ) << std::endl; }
        else { std::cout<<comma::join( comma::csv::names<plane_intersection::input1_t>(true), ',' ) << std::endl; }
        return; 
    }
    if(options.exists("--output-fields")){std::cout<<comma::join( comma::csv::names<plane_intersection::output_t>(false), ',' ) << std::endl; return; }
    if(options.exists("--output-format")){std::cout<<comma::csv::format::value<plane_intersection::output_t>() << std::endl; return; }
    boost::optional<std::string> plane_option=options.optional<std::string>("--plane");
    if(csv.fields.empty())
    {
        if(options.exists("--extended"))
            run<plane_intersection::line2_t, plane_intersection::plane1_t>(csv, plane_option);
        else
            run(csv, comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--normal",  "0,0,0")));
    }
    plane_intersection::fields_t fields(csv.fields);
    if(fields.contains("line/first") || fields.contains("line/second"))
        plane_intersection::step2<plane_intersection::line1_t>(fields, plane_option);
    else if(fields.contains("line") || fields.contains("line/dir"))//default
        plane_intersection::step2<plane_intersection::line2_t>(fields, plane_option);
    else
    {
        run(csv, comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--normal",  "0,0,0")));
    }
}
void plane_intersection::run( const comma::csv::options& csv_opt, const Eigen::Vector3d& normal)
{
    comma::csv::input_stream<plane_intersection::input1_t> is( std::cin, csv_opt, input1_t(normal));
    comma::csv::output_stream<plane_intersection::output_t> os( std::cout, csv_opt.binary() );
    comma::csv::tied<plane_intersection::input1_t,plane_intersection::output_t> tied(is,os);
    const double inf=std::numeric_limits<double>::infinity();
    while(is.ready() || std::cin.good())
    {
        const plane_intersection::input1_t* rec=is.read();
        if(!rec){break;}
        if(verbose)
        {
            std::cerr<<"p: "<<rec->point.x()<<", "<<rec->point.y()<<", "<<rec->point.z()<<std::endl;
            std::cerr<<"n: "<<rec->normal.x()<<", "<<rec->normal.y()<<", "<<rec->normal.z()<<std::endl;
        }
        double d=rec->point.dot(rec->normal);
        double n2=rec->normal.squaredNorm();
        if(verbose){std::cerr<<"d "<<d<<", n2 "<<n2<<std::endl;}
        //when d is 0 the line is parallel with the plane, so we return infinity
        plane_intersection::output_t out = (d == 0) ? Eigen::Vector3d(inf,inf,inf) : Eigen::Vector3d(( n2/d)*rec->point);
        tied.append(out);
    }
}
template<typename L, typename P>
void plane_intersection::run( const comma::csv::options& csv_opt, boost::optional<std::string> plane_option)
{
    P plane = plane_option ? comma::csv::ascii<P>().get(*plane_option) : P();
    comma::csv::input_stream<plane_intersection::input_t<L,P> > is( std::cin, csv_opt, plane_intersection::input_t<L,P> (plane));
    comma::csv::output_stream<plane_intersection::output_t> os( std::cout, csv_opt.binary() );
    comma::csv::tied<plane_intersection::input_t<L,P> ,plane_intersection::output_t> tied(is,os);
    const double inf=std::numeric_limits<double>::infinity();
    while(is.ready() || std::cin.good())
    {
        const plane_intersection::input_t<L,P> * rec=is.read();
        if(!rec){break;}
        if(verbose) { std::cerr<<"line: "<< rec->line_ << ";" <<" plane: "<< rec->plane_ <<std::endl; }
        double u=(rec->plane().point0() - rec->line().point0()).dot(rec->plane().normal());
        double d=rec->line().direction().dot(rec->plane().normal());
        if(verbose){std::cerr<<"u "<<u<<", d "<<d<<std::endl;}
        // if both d and u are 0 then the line is contained on the plane, we just return the first point of line
        plane_intersection::output_t out = (d == 0) ? ( (u == 0) ? rec->line().point0() : vector(inf,inf,inf))
            : vector(( u/d)*rec->line().direction() + rec->line().point0());
        tied.append(out);
    }
}
