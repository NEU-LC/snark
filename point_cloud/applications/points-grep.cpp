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
#include <string>

struct Bounds
{
    Bounds(): front(0), back(0), right(0), left(0), top(0), bottom(0) {}
    double front;
    double back;
    double right;
    double left;
    double top;
    double bottom;
};

struct BoundingPoint
{
    BoundingPoint(): x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    boost::posix_time::ptime timestamp;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct Point
{
    Point(): x(0), y(0), z(0), block(0), id(0), flag(1){}
    boost::posix_time::ptime timestamp;
    double x;
    double y;
    double z;
    comma::uint32 block;
    comma::uint32 id;
    comma::uint32 flag;
};

struct JoinedPoint
{
    Point p;
    BoundingPoint q;
};

namespace comma
{
    namespace visiting
    {
        template <> struct traits< Bounds >
        {
            template < typename K, typename V > static void visit( const K&, Bounds& p, V& v )
            {
                v.apply( "front", p.front );
                v.apply( "back", p.back );
                v.apply( "right", p.right );
                v.apply( "left", p.left );
                v.apply( "top", p.top );
                v.apply( "bottom", p.bottom );
            }

            template < typename K, typename V > static void visit( const K&, const Bounds& p, V& v )
            {
                v.apply( "front", p.front );
                v.apply( "back", p.back );
                v.apply( "right", p.right );
                v.apply( "left", p.left );
                v.apply( "top", p.top );
                v.apply( "bottom", p.bottom );
            }
        };
        template <> struct traits< Point >
        {
            template < typename K, typename V > static void visit( const K&, Point& p, V& v )
            {
                v.apply( "t", p.timestamp );
                v.apply( "x", p.x );
                v.apply( "y", p.y );
                v.apply( "z", p.z );
                v.apply( "block", p.block );
                v.apply( "id", p.id );
                v.apply( "flag", p.flag );
            }

            template < typename K, typename V > static void visit( const K&, const Point& p, V& v )
            {
                v.apply( "t", p.timestamp );
                v.apply( "x", p.x );
                v.apply( "y", p.y );
                v.apply( "z", p.z );
                v.apply( "block", p.block );
                v.apply( "id", p.id );
                v.apply( "flag", p.flag );
            }
        };
        template <> struct traits< BoundingPoint >
        {
            template < typename K, typename V > static void visit( const K&, BoundingPoint& p, V& v )
            {
                v.apply( "tt", p.timestamp );
                v.apply( "xx", p.x );
                v.apply( "yy", p.y );
                v.apply( "zz", p.z );
                v.apply( "roll", p.roll );
                v.apply( "pitch", p.pitch );
                v.apply( "yaw", p.yaw );
            }

            template < typename K, typename V > static void visit( const K&, const BoundingPoint& p, V& v )
            {
                v.apply( "tt", p.timestamp );
                v.apply( "xx", p.x );
                v.apply( "yy", p.y );
                v.apply( "zz", p.z );
                v.apply( "roll", p.roll );
                v.apply( "pitch", p.pitch );
                v.apply( "yaw", p.yaw );
            }
        };
        template <> struct traits< JoinedPoint >
        {
            template < typename K, typename V > static void visit( const K&, JoinedPoint& p, V& v )
            {
                v.apply( "", p.p );
                v.apply( "", p.q );
            }
            template < typename K, typename V > static void visit( const K&, const JoinedPoint& p, V& v )
            {
                v.apply( "", p.p );
                v.apply( "", p.q );
            }
        };
    }
}

// todo (Seva)
// - rename to points-grep
// - usage: points-grep <operation>
// - usage: points-grep stream
// - usage: points-grep shape (when input is points joined with shapes)

// usage:
// cat points.csv | points-grep stream --fields=t,x,y,z,block,id,flag "nav.csv;fields=tt,xx,yy,zz,roll,pitch,yaw"
// cat points.csv | points-grep shape --fields=t,x,y,z,block,id,flag,tt,xx,yy,zz,roll,pitch,yaw

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
    std::cerr << "      <stream>: example: \"nav.csv;fields=tt,xx,yy,zz,roll,pitch,yaw\" " << std::endl;
    std::cerr << std::endl;
    std::cerr << "  shape: points are assumed to be joined with their positions" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>:" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      --bounds=<front>,<back>,<right>,<left>,<top>,<bottom>" << std::endl;
    std::cerr << "      --offset=<offset> global error offset from object boundaries" << std::endl;
    std::cerr << "      --output-all: output all points" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

static double offset;
static Bounds bounds;

void filter_point(JoinedPoint& pq)
{
    //get polygon from bounds
    Eigen::MatrixXd origins(3,7); //origin, front, back, right, left, top, bottom


    origins<<0, bounds.front + offset, -bounds.back - offset, 0, 0, 0, 0,
             0, 0, 0, bounds.right + offset, -bounds.left - offset, 0, 0,
             0, 0, 0, 0, 0, -bounds.top - offset, bounds.bottom + offset;


    //convert vehicle bounds to world coordinates
    //get transformation

    Eigen::MatrixXd rot_matrix=snark::rotation_matrix::rotation( pq.q.roll, pq.q.pitch, pq.q.yaw );
    Eigen::Vector3d trans_vector(pq.q.x, pq.q.y, pq.q.z);

    for(int cntr2=0; cntr2<origins.cols(); cntr2++)
    {
        origins.col(cntr2)=rot_matrix*origins.col(cntr2)+trans_vector;
    }

    //get plane equations
    Eigen::MatrixXd A(6,3);
    Eigen::VectorXd b(6);

    for(int cntr2=1; cntr2<origins.cols(); cntr2++)
    {
        A.row(cntr2-1)<<origins.col(0).transpose()-origins.col(cntr2).transpose();
        b(cntr2-1)=origins.col(cntr2).transpose()*(origins.col(0)-origins.col(cntr2));
    }


    Eigen::Vector3d point(pq.p.x,pq.p.y,pq.p.z);

    //check polygon in 3d
    // use if statement not assignment because point might already be filtered
    if(snark::geometry::convex_polytope(A,b).has(point))
    {
        pq.p.flag=0;
    }
}

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv );
    if( options.exists( "--help" ) || options.exists( "-h" ) || argc == 1 ) { usage(); }
    comma::csv::options csv(options);

    offset=options.value("--offset",0.5);

    bounds=comma::csv::ascii<Bounds>().get(options.value("--bounds",std::string("0,0,0,0,0,0")));

    bool output_all = options.exists( "--output-all");
    comma::csv::input_stream<JoinedPoint> istream(std::cin,csv);
    comma::csv::output_stream<JoinedPoint> ostream(std::cout,csv);

    std::vector<std::string> fields=comma::split(csv.fields,',');

    //check if flag field exists
    bool flag_exists=false;
    for(unsigned int cntr=0; cntr<fields.size(); cntr++)
    {
        if(fields[cntr]=="flag")
        {
            flag_exists=true;
            break;
        }
    }

    std::vector<std::string> unnamed=options.unnamed("--output-all","--binary,--fields,--offset,--bounds");

    std::string operation=unnamed[0];

    //bounding stream
    std::deque<BoundingPoint> bounding_queue;
    boost::shared_ptr< comma::io::istream > bounding_is;
    boost::shared_ptr< comma::csv::input_stream<BoundingPoint> > bounding_istream;

    comma::io::select istream_select;
    comma::io::select bounding_istream_select;

    if(operation=="stream")
    {
        if(unnamed.size()<2){ usage(); }
        bounding_is.reset(new comma::io::istream( comma::split(unnamed[1],';')[0] )); // get stream name

        comma::name_value::parser parser( "filename" );
        comma::csv::options bounding_csv = parser.get< comma::csv::options >( unnamed[1] ); // get stream options
        bounding_istream.reset(new comma::csv::input_stream<BoundingPoint>(**bounding_is, bounding_csv)); // create stream

        istream_select.read().add(0);
        istream_select.read().add(bounding_is->fd());
        bounding_istream_select.read().add(bounding_is->fd());
    }
    else if(operation=="shape")
    {
        istream_select.read().add(0);
    }

    comma::signal_flag is_shutdown;

    JoinedPoint pq;
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
                const BoundingPoint* q = bounding_istream->read();
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
                const JoinedPoint* pq_ptr = istream.read();
                if( !pq_ptr ) { break; }
                pq=*pq_ptr;
            }

            //get bound
            while(bounding_queue.size()>=2)
            {
                if( pq.p.timestamp < bounding_queue[1].timestamp ) { break; }
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
            if(pq.p.timestamp < bounding_queue[0].timestamp)
            {
                continue;
            }

            //match
            bool is_first=( pq.p.timestamp - bounding_queue[0].timestamp < bounding_queue[1].timestamp - pq.p.timestamp );
            pq.q = is_first ? bounding_queue[0] : bounding_queue[1]; // assign bounding point

        }
        else if(operation=="shape")
        {
            bool istream_ready=istream.ready();

            if(!istream_ready)
            {
                istream_select.wait(boost::posix_time::milliseconds(10));
                if(istream_select.read().ready(0))
                {
                   istream_ready=true;
                }
            }

            if(!istream_ready) { continue; }

            const JoinedPoint* pq_ptr = istream.read();

            if( !pq_ptr ) { break; }

            pq=*pq_ptr;
        }

        //filter out object points
        filter_point(pq);

        if(!pq.p.flag && !output_all)
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
            std::cout.write( reinterpret_cast< const char* >( &pq.p.flag ), sizeof( Point::flag ) );
        }
        else
        {
            std::string line=comma::join( istream.ascii().last(), csv.delimiter );
            line+=","+boost::lexical_cast<std::string>(pq.p.flag);
            ostream.write(pq,line);
        }
        ostream.flush();

    }

    return(0);
}
