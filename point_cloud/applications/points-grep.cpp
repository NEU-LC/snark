#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/csv/binary.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include <snark/visiting/eigen.h>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <snark/math/rotation_matrix.h>
#include <snark/math/geometry/polytope.h>
#include <snark/math/applications/frame.h>
#include <string>

// todo
// - --help: add examples
// - --help: --offset: what is it supposed to be?

struct bounds_t
{
    bounds_t(): front(0), back(0), right(0), left(0), top(0), bottom(0) {}
    double front;
    double back;
    double right;
    double left;
    double top;
    double bottom;
};

struct point
{
    point(): block(0), id(0), flag(1){}
    boost::posix_time::ptime timestamp;
    Eigen::Vector3d coordinates;
    comma::uint32 block;
    comma::uint32 id;
    comma::uint32 flag;
};

typedef snark::applications::frame::position_type bounding_point;

struct joined_point
{
    point bounded;
    bounding_point bounding;
};

namespace comma
{
    namespace visiting
    {
        template <> struct traits< bounds_t >
        {
            template < typename K, typename V > static void visit( const K&, bounds_t& p, V& v )
            {
                v.apply( "front", p.front );
                v.apply( "back", p.back );
                v.apply( "right", p.right );
                v.apply( "left", p.left );
                v.apply( "top", p.top );
                v.apply( "bottom", p.bottom );
            }

            template < typename K, typename V > static void visit( const K&, const bounds_t& p, V& v )
            {
                v.apply( "front", p.front );
                v.apply( "back", p.back );
                v.apply( "right", p.right );
                v.apply( "left", p.left );
                v.apply( "top", p.top );
                v.apply( "bottom", p.bottom );
            }
        };
        template <> struct traits< point >
        {
            template < typename K, typename V > static void visit( const K&, point& p, V& v )
            {
                v.apply( "t", p.timestamp );
                v.apply( "coordinates", p.coordinates );
                v.apply( "block", p.block );
                v.apply( "id", p.id );
                v.apply( "flag", p.flag );
            }

            template < typename K, typename V > static void visit( const K&, const point& p, V& v )
            {
                v.apply( "t", p.timestamp );
                v.apply( "coordinates", p.coordinates );
                v.apply( "block", p.block );
                v.apply( "id", p.id );
                v.apply( "flag", p.flag );
            }
        };
        
        template <> struct traits< joined_point >
        {
            template < typename K, typename V > static void visit( const K&, joined_point& p, V& v )
            {
                v.apply( "bounded", p.bounded );
                v.apply( "bounding", p.bounding );
            }
            template < typename K, typename V > static void visit( const K&, const joined_point& p, V& v )
            {
                v.apply( "bounded", p.bounded );
                v.apply( "bounding", p.bounding );
            }
        };
    }
}

// todo (Seva)
// - rename to points-grep
// - usage: points-grep <operation>
// - usage: points-grep stream
// - usage: points-grep shape (when input is points joined with shapes)

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "time-match and filter points from dynamic objects" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "output: each point is tagged with a flag of type ui" << std::endl;
    std::cerr<< "        the value of flag is 0 for filtered points and 1 otherwise" << std::endl;
    std::cerr<< "        if --output-all is not specified only non-filtered points are output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-grep <operation> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<operation>:" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  stream <stream>: points are time-joined with a position stream and filtered based on a bounding box around the positions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      <stream>: example: \"nav.csv;fields=t,x,y,z,roll,pitch,yaw\" " << std::endl;
    std::cerr << "      fields" << std::endl;
    std::cerr << "          bounded: " << comma::join( comma::csv::names< point >( false ), ',' ) << std::endl;
    std::cerr << "                   default: t,x,y,z" << std::endl;
    std::cerr << "          bounding: " << comma::join( comma::csv::names< bounding_point >( false ), ',' ) << std::endl;
    std::cerr << "                    or shorthand: bounded,bounding" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  shape: points are assumed to be joined with their positions" << std::endl;
    std::cerr << "      fields: " << comma::join( comma::csv::names< joined_point >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>:" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      --bounds=<front>,<back>,<right>,<left>,<top>,<bottom>" << std::endl;
    std::cerr << "      --offset=<offset> error margin value added to bounds (for user convenience)" << std::endl;
    std::cerr << "      --output-all: output all points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat points.csv | points-grep stream --fields=bounded/t,bounded/coordinates,bounded/block,bounded/id,bounded/flag \"nav.csv;fields=t,x,y,z,roll,pitch,yaw\" --bounds=1.0,2.0,1.0,2.0,1.0,2.0 --offset=0.1" << std::endl;
    std::cerr << "    cat points.csv | points-grep shape --fields=bounded/t,bounded/coordinates,bounded/block,bounded/id,bounded/flag,bounding --bounds=1.0,2.0,1.0,2.0,1.0,2.0 --offset=0.1" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static double offset;
static bounds_t bounds;

void filter_point(joined_point& pq)
{
    //get polygon from bounds
    Eigen::MatrixXd origins(3,7); //origin, front, back, right, left, top, bottom


    origins<<0, bounds.front + offset, -bounds.back - offset, 0, 0, 0, 0,
             0, 0, 0, bounds.right + offset, -bounds.left - offset, 0, 0,
             0, 0, 0, 0, 0, -bounds.top - offset, bounds.bottom + offset;


    //convert vehicle bounds to world coordinates
    //get transformation

    Eigen::MatrixXd rot_matrix = snark::rotation_matrix::rotation(pq.bounding.value.orientation);

    for(int cntr2=0; cntr2<origins.cols(); cntr2++)
    {
        origins.col(cntr2)=rot_matrix*origins.col(cntr2)+pq.bounding.value.coordinates;
    }

    //get plane equations
    Eigen::MatrixXd A(6,3);
    Eigen::VectorXd b(6);

    for(int cntr2=1; cntr2<origins.cols(); cntr2++)
    {
        A.row(cntr2-1)<<origins.col(0).transpose()-origins.col(cntr2).transpose();
        b(cntr2-1)=origins.col(cntr2).transpose()*(origins.col(0)-origins.col(cntr2));
    }

    //check polygon in 3d
    // use if statement not assignment because point might already be filtered
    if(snark::geometry::convex_polytope(A,b).has(pq.bounded.coordinates))
    {
        pq.bounded.flag=0;
    }
}

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv );
    if( options.exists( "--help" ) || options.exists( "-h" ) || argc == 1 ) { usage(); }
    comma::csv::options csv(options);
    //if( csv.fields.empty() ) { csv.fields="t,x,y,z"; }
    csv.full_xpath=true;
    offset=options.value("--offset",0.5);

    bounds=comma::csv::ascii<bounds_t>().get(options.value("--bounds",std::string("0,0,0,0,0,0")));

    bool output_all = options.exists( "--output-all");
    std::vector<std::string> unnamed=options.unnamed("--output-all,--verbose,-v","-.*");

    std::string operation=unnamed[0];

    bool flag_exists = false;
    if( operation == "stream" )
    {
        csv.full_xpath = false;
        if( csv.fields.empty() ) { csv.fields = "t,x,y,z"; }
        flag_exists = csv.has_field( "flag" );
    }
    else if( operation == "shape" )
    {
        flag_exists = csv.has_field( "bounded/flag" );
    }
    else
    {
        std::cerr << "points-grep: expected operation, got: \"" << operation << "\"" << std::endl;
        return 1;
    }
    
    comma::csv::input_stream<joined_point> istream(std::cin,csv);
    comma::csv::output_stream<joined_point> ostream(std::cout,csv);
    
    //bounding stream
    std::deque<bounding_point> bounding_queue;
    boost::shared_ptr< comma::io::istream > bounding_is;
    boost::shared_ptr< comma::csv::input_stream<bounding_point> > bounding_istream;

    comma::io::select istream_select;
    comma::io::select bounding_istream_select;

    if(operation=="stream")
    {
        if(unnamed.size()<2){ usage(); }
        bounding_is.reset(new comma::io::istream( comma::split(unnamed[1],';')[0] )); // get stream name

        comma::name_value::parser parser( "filename" );
        comma::csv::options bounding_csv = parser.get< comma::csv::options >( unnamed[1] ); // get stream options
        bounding_istream.reset(new comma::csv::input_stream<bounding_point>(**bounding_is, bounding_csv)); // create stream

        istream_select.read().add(0);
        istream_select.read().add(bounding_is->fd());
        bounding_istream_select.read().add(bounding_is->fd());
    }
    else if(operation=="shape")
    {
        istream_select.read().add(0);
    }
    else
    {
        std::cerr << "points-grep: expected operation, got: \"" << operation << "\"" << std::endl;
        return 1;
    }

    comma::signal_flag is_shutdown;

    joined_point pq;
    bool next=true;

    while(!is_shutdown && ( istream.ready() || ( std::cin.good() && !std::cin.eof() ) ))
    {
        if(operation=="stream")
        {
            bool bounding_data_available =  bounding_istream->ready() || ( (*bounding_is)->good() && !(*bounding_is)->eof());

            //check so we do not block
            bool bounding_istream_ready=bounding_istream->ready();
            bool istream_ready=istream.ready();

            if(next)
            {
                //only check istream if we need a new point
                if(!bounding_istream_ready || !istream_ready)
                {
                   if(!bounding_istream_ready && !istream_ready)
                   {
                       istream_select.wait(boost::posix_time::milliseconds(10));
                   }
                   else
                   {
                       istream_select.check();
                   }
                   if(istream_select.read().ready(bounding_is->fd()))
                   {
                       bounding_istream_ready=true;
                   }
                   if(istream_select.read().ready(0))
                   {
                       istream_ready=true;
                   }
                }
            }
            else
            {
               if(!bounding_istream_ready)
               {
                   bounding_istream_select.wait(boost::posix_time::milliseconds(10));
                   if(bounding_istream_select.read().ready(bounding_is->fd()))
                   {
                       bounding_istream_ready=true;
                   }
               }
            }

            //keep storing available bounding data
            if(bounding_istream_ready)
            {
                const bounding_point* q = bounding_istream->read();
                if( q )
                {
                    bounding_queue.push_back(*q);
                }
                else
                {
                    bounding_data_available=false;
                }
            }

            //if we are done with the last bounded point get next
            if(next)
            {
                if(!istream_ready) { continue; }
                const joined_point* pq_ptr = istream.read();
                if( !pq_ptr ) { break; }
                pq=*pq_ptr;
            }

            //get bound
            while(bounding_queue.size()>=2)
            {
                if( pq.bounded.timestamp < bounding_queue[1].t ) { break; }
                bounding_queue.pop_front();
            }

            if(bounding_queue.size()<2)
            {
                //bound not found
                //do we have more data?
                if(!bounding_data_available) { break; }
                next=false;
                continue;
            }

            //bound available
            next=true; //get new point on next iteration

            //discard late points
            if(pq.bounded.timestamp < bounding_queue[0].t)
            {
                continue;
            }

            //match
            bool is_first=( pq.bounded.timestamp - bounding_queue[0].t < bounding_queue[1].t - pq.bounded.timestamp );
            pq.bounding = is_first ? bounding_queue[0] : bounding_queue[1]; // assign bounding point

        }
        else if(operation=="shape")
        {
//             bool istream_ready=istream.ready();
// 
//             if(!istream_ready)
//             {
//                 istream_select.wait(boost::posix_time::milliseconds(10));
//                 if(istream_select.read().ready(0))
//                 {
//                    istream_ready=true;
//                 }
//             }
// 
//             if(!istream_ready) { continue; }

            const joined_point* pq_ptr = istream.read();

            if( !pq_ptr ) { break; }

            pq=*pq_ptr;
        }

        //filter out object points
        filter_point(pq);

        if(!pq.bounded.flag && !output_all)
        {
            continue;
        }

        if(flag_exists)
        {
            ostream.write(pq);
            ostream.flush();
            continue;
        }

        //append flag
        if(ostream.is_binary())
        {
            ostream.write(pq,istream.binary().last());
            std::cout.write( reinterpret_cast< const char* >( &pq.bounded.flag ), sizeof( point::flag ) );
        }
        else
        {
            std::string line=comma::join( istream.ascii().last(), csv.delimiter );
            line+=","+boost::lexical_cast<std::string>(pq.bounded.flag);
            ostream.write(pq,line);
        }
        ostream.flush();

    }

    return(0);
}
