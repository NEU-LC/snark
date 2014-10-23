#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/csv/binary.h>
#include <comma/io/select.h>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <vector>
#include <Eigen/Dense>

struct Point
{
    Point(): x(0), y(0), z(0), block(0), id(0){}
    boost::posix_time::ptime timestamp;
    double x;
    double y;
    double z;
    comma::uint32 block;
    comma::uint32 id;
    double size;
};

static bool outputsize=false;

namespace comma
{
    namespace visiting
    {
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
                v.apply( "size", p.size );
            }

            template < typename K, typename V > static void visit( const K&, const Point& p, V& v )
            {
                v.apply( "t", p.timestamp );
                v.apply( "x", p.x );
                v.apply( "y", p.y );
                v.apply( "z", p.z );
                v.apply( "block", p.block );
                v.apply( "id", p.id );
                v.apply( "size", p.size );
            }
        };
    }
}

typedef std::vector< std::pair<Point,std::string> > block_t;

void partitions_to_centroids(block_t& block, block_t& centroids, std::vector<double>& sizes)
{
    block_t::iterator itr;

    while(!block.empty())
    {
        itr=block.begin();
        centroids.push_back(*itr); // store other information

        comma::uint32 id=itr->first.id;

        Eigen::MatrixXd Points(3,1);
        Points(0,0)=itr->first.x;
        Points(1,0)=itr->first.y;
        Points(2,0)=itr->first.z;

        block.erase(itr);

        while(itr!=block.end())
        {
            if(itr->first.id==id)
            {
                Points.conservativeResize(Eigen::NoChange,Points.cols()+1);
                Points(0,Points.cols()-1)=itr->first.x;
                Points(1,Points.cols()-1)=itr->first.y;
                Points(2,Points.cols()-1)=itr->first.z;

                block.erase(itr);
            }
            else
            {
                itr++;
            }
        }

        //get centroid
        Eigen::MatrixXd Mean=Points.rowwise().mean();
        centroids.back().first.x=Mean(0,0);
        centroids.back().first.y=Mean(1,0);
        centroids.back().first.z=Mean(2,0);

        //get bounding box
        Eigen::MatrixXd Mins=Points.rowwise().minCoeff();
        Eigen::MatrixXd Maxs=Points.rowwise().maxCoeff();
        Eigen::MatrixXd Extents=Maxs-Mins;
        sizes.push_back(Extents.maxCoeff());
    }
}


void publish_centroids(const block_t& centroids, const std::vector<double>& sizes, comma::csv::output_stream<Point>& ostream)
{
    //publish centroids
    for(unsigned int cntr=0; cntr<centroids.size(); cntr++)
    {
        if(ostream.is_binary())
        {
            ostream.write(centroids.at(cntr).first,centroids.at(cntr).second);
            if(outputsize)
            {
                std::cout.write( reinterpret_cast< const char* >( &sizes.at(cntr) ), sizeof( double ) );
            }
        }
        else
        {
            std::string line=centroids.at(cntr).second;
            if(outputsize)
            {
                line+=","+boost::lexical_cast<std::string>(sizes.at(cntr));
            }
            ostream.write(centroids.at(cntr).first,line);
        }
    }
    ostream.flush();
}

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv );
    if( options.exists( "--help,-h" ) )
    {
        std::cerr<<"gets the centroid of a partitioned scan based on id"<<std::endl;
        std::cerr<<"parititions do not have to be in sequence"<<std::endl;
    }
    outputsize=options.exists("--output-size");
    comma::csv::options csv( options );
    comma::csv::input_stream<Point> istream(std::cin,csv);
    comma::csv::output_stream<Point> ostream(std::cout,csv);
    comma::io::select select;
    select.read().add(0);

    comma::signal_flag is_shutdown;



    //initialise so that first point is interpreted as a new block
    comma::uint32 last_block = std::numeric_limits< comma::uint32 >::max();


    std::vector< std::pair<Point,std::string> > block;

    while(!is_shutdown && std::cin.good() && !std::cin.eof())
    {

        if(!istream.ready())
        {
            select.wait(1);
            if(!select.read().ready(0))
            {
                continue;
            }
        }
        const Point* input=istream.read();
        block_t centroids;
        std::vector<double> sizes;
        if(!input)
        {
            //parition last block
            if(!block.empty())
            {
                partitions_to_centroids(block,centroids,sizes);
                publish_centroids(centroids,sizes,ostream);
            }
            return(0);
        }
        if(input->block != last_block && !block.empty())
        {
            partitions_to_centroids(block,centroids,sizes);
            publish_centroids(centroids,sizes,ostream);
        }


        std::string line;
        if( csv.binary() )
        {
            line.resize( csv.format().size() );
            ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
        }
        else
        {
            line = comma::join( istream.ascii().last(), csv.delimiter );
        }
        std::pair<Point,std::string> input_pair = std::make_pair( *input, line );

        block.push_back(input_pair);
        last_block=input->block;
    }

    return(0);
}

