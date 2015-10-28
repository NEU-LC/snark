#ifndef DETAIL_VECTOR_CALC_H
#define DETAIL_VECTOR_CALC_H
#include <iostream>
#include <comma/csv/stream.h>
#include <snark/visiting/eigen.h>

extern comma::csv::options csv;
extern bool verbose;

struct vector_calc
{
    typedef Eigen::Vector3d vector;
    static void usage()
    {
        std::cerr << "    vector calculations: following operations perform calculations on 3d vectors and scalars" << std::endl;
        std::cerr << "        common options:" << std::endl;
        std::cerr << "            --input-fields: print input field names and exit" << std::endl;
        std::cerr << "            --output-fields: print output field names and exit" << std::endl;
        std::cerr << "            --output-format: print output format and exit" << std::endl;
        std::cerr << "    " << std::endl;
        std::cerr << "        cross: calculate vector cross product; output: vector" << std::endl;
        std::cerr << "            input fields: v, u" << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
        std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        dot: calculate vector dot product; output: scalar" << std::endl;
        std::cerr << "            input fields: v, u" << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
        std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        norm: calculate vector's norm " << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector" << std::endl;
        std::cerr << "            --square: calculate norm square" << std::endl;
        std::cerr << "            --invert: calculate invert of norm; use with --square for invert of norm square" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        scale: calculate multiplication of scalar and vector;output vector" << std::endl;
        std::cerr << "            input fields:  v,a (scalar)" << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector" << std::endl;
        std::cerr << "            --scalar: default value for a" << std::endl;
        std::cerr << "            --invert: invert the scalar first and then multiply it by vector, i.e. (1/a)*v" << std::endl;
        std::cerr << "" << std::endl;
        std::cerr << "        add: add two vectors: v + u " << std::endl;
        std::cerr << "            input fields: v, u" << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
        std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        subtract: subtract two vectors: u - v " << std::endl;
        std::cerr << "            input fields: v, u" << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
        std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
        std::cerr << std::endl;
        std::cerr << "        normal: calculate normal vector given three points as: (w-v) x (u-v)" << std::endl;
        std::cerr << "            input fields: v, u, w" << std::endl;
        std::cerr << "            --v=<x>,<y>,<z>: default value for vector 1" << std::endl;
        std::cerr << "            --u=<x>,<y>,<z>: default value for vector 2" << std::endl;
        std::cerr << "            --w=<x>,<y>,<z>: default value for vector 3" << std::endl;
        std::cerr << "            --unit: normalize the output to unit length" << std::endl;
        std::cerr << std::endl;
    }
    static void usage_list_operations()
    {
        std::cerr << "    cross" << std::endl;
        std::cerr << "    dot" << std::endl;
        std::cerr << "    norm" << std::endl;
        std::cerr << "    scale" << std::endl;
        std::cerr << "    add" << std::endl;
        std::cerr << "    subtract" << std::endl;
        std::cerr << "    normal" << std::endl;
    }
    static bool has_operation(const std::string& operation)
    {
        return (operation=="cross") || (operation=="dot") || (operation=="norm") || (operation=="scale") || (operation=="add") || (operation=="subtract") || (operation=="normal");
    }
    static void process(const std::string& operation, const comma::command_line_options& options)
    {
        if(operation=="cross")
        {
            if(options.exists("--input-fields")){vector_calc::cross().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::cross().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::cross().output_format(); return; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d v2_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            vector_calc::cross(v1_default,v2_default).run(csv);
            return;
        }
        if(operation=="dot")
        {
            if(options.exists("--input-fields")){vector_calc::dot().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::dot().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::dot().output_format(); return; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d v2_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            vector_calc::dot(v1_default,v2_default).run(csv);
            return;
        }
        if(operation=="norm")
        {
            if(options.exists("--input-fields")){vector_calc::norm().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::norm().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::norm().output_format(); return; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            vector_calc::norm(v1_default,options.exists("--square"),options.exists("--invert")).run(csv);
            return;
        }
        if(operation=="scale")
        {
            if(options.exists("--input-fields")){vector_calc::scale().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::scale().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::scale().output_format(); return; }
            Eigen::Vector3d v=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            vector_calc::scalar s=comma::csv::ascii< double >().get(options.value< std::string >( "--scalar", "0"));
            vector_calc::scale(v, s, options.exists("--invert")).run(csv);
            return;
        }
        if(operation=="subtract")
        {
            if(options.exists("--input-fields")){vector_calc::subtract().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::subtract().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::subtract().output_format(); return; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d v2_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            vector_calc::subtract(v1_default,v2_default).run(csv);
            return;
        }
        if(operation=="add")
        {
            if(options.exists("--input-fields")){vector_calc::add().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::add().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::add().output_format(); return; }
            Eigen::Vector3d v1_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d v2_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            vector_calc::add(v1_default,v2_default).run(csv);
            return;
        }
        if(operation=="normal")
        {
            if(options.exists("--input-fields")){vector_calc::normal().input_fields(); return; }
            if(options.exists("--output-fields")){vector_calc::normal().output_fields(); return; }
            if(options.exists("--output-format")){vector_calc::normal().output_format(); return; }
            Eigen::Vector3d v_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--v",  "0,0,0"));
            Eigen::Vector3d u_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--u",  "0,0,0"));
            Eigen::Vector3d w_default=comma::csv::ascii< Eigen::Vector3d >().get(options.value< std::string >( "--w",  "0,0,0"));
            vector_calc::normal(v_default,u_default,w_default,options.exists("--unit")).run(csv);
            return;
        }
        COMMA_THROW( comma::exception, "vector_calc operation not supported :" << operation );
    }
    //typedef double scalar;
    static vector vzero;
    template<typename input_t,typename output_t>
    struct operation_t
    {
        virtual output_t calc(const input_t& in)=0;
        input_t default_input;
        operation_t(const input_t& def):default_input(def){}
        operation_t(){}
        void run(const comma::csv::options& csv_opt)
        {
            comma::csv::input_stream<input_t> is( std::cin, csv_opt, default_input);
            comma::csv::output_stream<output_t> os( std::cout, csv_opt.binary() );
            comma::csv::tied<input_t,output_t> tied(is,os);
            while(is.ready() || std::cin.good())
            {
                const input_t* rec=is.read();
                if(!rec){break;}
                tied.append(calc(*rec));
            }
        }
        void input_fields(){std::cout<<comma::join( comma::csv::names<input_t>(true), ',' ) << std::endl; }
        void output_fields(){std::cout<<comma::join( comma::csv::names<output_t>(false), ',' ) << std::endl; }
        void output_format(){std::cout<<comma::csv::format::value<output_t>() << std::endl; }
    };
    struct vector_pair
    {
        vector v;
        vector u;
        vector_pair():v(vzero),u(vzero){}
        vector_pair(const vector& d1,const vector& d2):v(d1),u(d2){}
    };
    struct vector_triple
    {
        vector v;
        vector u;
        vector w;
        vector_triple():v(vzero),u(vzero),w(vzero){}
        vector_triple(const vector& d1,const vector& d2,const vector& d3):v(d1),u(d2),w(d3){}
    };
    struct scalar
    {
        double a;
        scalar():a(0){}
        scalar(double d):a(d){}
    };
    struct cross:operation_t<vector_pair,vector>
    {
        cross(){}
        cross(const vector& default1, const vector& default2 ):operation_t(vector_pair(default1,default2)){}
        vector calc(const vector_pair& in)
        {
            return in.v.cross(in.u);
        }
    };
    struct dot:operation_t<vector_pair,scalar>
    {
        dot(){}
        dot(const vector& default1, const vector& default2 ):operation_t(vector_pair(default1,default2)){}
        scalar calc(const vector_pair& in)
        {
            return in.v.dot(in.u);
        }
    };
    struct norm:operation_t<vector,scalar>
    {
        norm(){}
        norm(const vector& default1, bool squre=false, bool invert=false ) :
            operation_t(default1)
            ,_squre(squre)
            ,_invert(invert)
        {
        }
        scalar calc(const vector& in)
        {
            double d=_squre?in.squaredNorm(): in.norm();
            return _invert?1/d:d;
        }
        bool _squre;
        bool _invert;
    };
    typedef std::pair<vector,scalar> vector_scalarpair;
    struct scale:operation_t<vector_scalarpair,vector>
    {
        scale(){}
        scale(const vector& v, const scalar& s, bool invert=false ) :
            operation_t(vector_scalarpair(v,s))
            ,_invert(invert)
        {
        }
        vector calc(const vector_scalarpair& in)
        {
            double d=_invert?1/in.second.a:in.second.a;
            return d*in.first;
        }
        bool _invert;
    };
    struct add:operation_t<vector_pair,vector>
    {
        add(){}
        add(const vector& default1, const vector& default2 ):operation_t(vector_pair(default1,default2)){}
        vector calc(const vector_pair& in)
        {
            return in.v+in.u;
        }
    };
    struct subtract:operation_t<vector_pair,vector>
    {
        subtract(){}
        subtract(const vector& default1, const vector& default2 ):operation_t(vector_pair(default1,default2)){}
        vector calc(const vector_pair& in)
        {
            return in.u-in.v;
        }
    };
    struct normal:operation_t<vector_triple,vector>
    {
        bool unit_;
        normal(){}
        normal(const vector& default1, const vector& default2, const vector& default3, bool unit ):operation_t(vector_triple(default1,default2,default3)),unit_(unit){}
        vector calc(const vector_triple& in)
        {
            vector normal=(in.u-in.v).cross(in.w-in.v);
            if(unit_) { normal /= normal.norm(); }
            return normal;
        }
    };
};
vector_calc::vector vector_calc::vzero=Eigen::Vector3d::Zero();

namespace comma { namespace visiting {


template <> struct traits< vector_calc::vector_pair >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::vector_pair& t, V& v )
    {
        v.apply( "v", t.v);
        v.apply( "u", t.u);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::vector_pair& t, V& v )
    {
        v.apply( "v", t.v);
        v.apply( "u", t.u);
    }
};

template <> struct traits< vector_calc::vector_triple >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::vector_triple& t, V& v )
    {
        v.apply( "v", t.v);
        v.apply( "u", t.u);
        v.apply( "w", t.w);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::vector_triple& t, V& v )
    {
        v.apply( "v", t.v);
        v.apply( "u", t.u);
        v.apply( "w", t.w);
    }
};

template <> struct traits< vector_calc::vector_scalarpair >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::vector_scalarpair& t, V& v )
    {
        v.apply( "v", t.first);
        v.apply( "scalar", t.second.a);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::vector_scalarpair& t, V& v )
    {
        v.apply( "v", t.first);
        v.apply( "scalar", t.second.a);
    }
};

template <> struct traits< vector_calc::scalar >
{
    template< typename K, typename V > static void visit( const K& k, vector_calc::scalar& t, V& v )
    {
        v.apply( "scalar", t.a);
    }
    template< typename K, typename V > static void visit( const K& k, const vector_calc::scalar& t, V& v )
    {
        v.apply( "scalar", t.a);
    }
};

} } // namespace comma { namespace visiting {
    
#endif