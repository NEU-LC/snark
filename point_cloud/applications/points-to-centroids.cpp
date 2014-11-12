#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/csv/binary.h>
#include <boost/tokenizer.hpp>

struct point
{
    point(): x(0), y(0), z(0), block(0), id(0){}
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
        template <> struct traits< point >
        {
            template < typename K, typename V > static void visit( const K&, point& p, V& v )
            {
                v.apply( "t", p.timestamp );
                v.apply( "x", p.x );
                v.apply( "y", p.y );
                v.apply( "z", p.z );
                v.apply( "block", p.block );
                v.apply( "id", p.id );
                v.apply( "size", p.size );
            }

            template < typename K, typename V > static void visit( const K&, const point& p, V& v )
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

typedef std::vector< std::pair<point,std::string> > block_t;

void partitions_to_centroids(block_t& block, block_t& centroids, std::vector<double>& sizes)
{
    block_t::iterator itr;

    while(!block.empty())
    {
        itr=block.begin();
        centroids.push_back(*itr); // store other information

        comma::uint32 id=itr->first.id;

        Eigen::MatrixXd points(3,1);
        points(0,0)=itr->first.x;
        points(1,0)=itr->first.y;
        points(2,0)=itr->first.z;

        block.erase(itr);

        while(itr!=block.end())
        {
            if(itr->first.id==id)
            {
                points.conservativeResize(Eigen::NoChange,points.cols()+1);
                points(0,points.cols()-1)=itr->first.x;
                points(1,points.cols()-1)=itr->first.y;
                points(2,points.cols()-1)=itr->first.z;

                block.erase(itr);
            }
            else
            {
                itr++;
            }
        }

        //get centroid
        Eigen::MatrixXd Mean=points.rowwise().mean();
        centroids.back().first.x=Mean(0,0);
        centroids.back().first.y=Mean(1,0);
        centroids.back().first.z=Mean(2,0);

        //get bounding box
        Eigen::MatrixXd Mins=points.rowwise().minCoeff();
        Eigen::MatrixXd Maxs=points.rowwise().maxCoeff();
        Eigen::MatrixXd Extents=Maxs-Mins;
        sizes.push_back(Extents.maxCoeff());
    }
}


void publish_centroids(const block_t& centroids, const std::vector<double>& sizes, comma::csv::output_stream<point>& ostream)
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
    try
    {
        comma::command_line_options options( argc, argv );
        if( options.exists( "--help,-h" ) )
        {
            std::cerr << "gets the centroid of a partitioned point cloud based on id" << std::endl;
            std::cerr << "parititions do not have to be in sequence"<<std::endl;
            std::cerr << std::endl;
            std::cerr << "usage: cat points.csv | points-to-centroids [<options>]" << std::endl;
            std::cerr << std::endl;
            std::cerr << "input: partitioned point cloud where field id corresponds to the partition number" << std::endl;
            std::cerr << std::endl;
            std::cerr << "output: the centroid of each partition with a size field optionally appended" << std::endl;
            std::cerr << std::endl;
            std::cerr << "<options>" << std::endl;
            std::cerr << "    --output-size: if present output partition size" << std::endl;
            std::cerr << comma::csv::options::usage() << std::endl;
            std::cerr << std::endl;
            exit(-1);
        }
        outputsize=options.exists("--output-size");
        comma::csv::options csv( options );
        comma::csv::input_stream<point> istream(std::cin,csv);
        comma::csv::output_stream<point> ostream(std::cout,csv);
        comma::signal_flag is_shutdown;



        //initialise so that first point is interpreted as a new block
        comma::uint32 last_block = std::numeric_limits< comma::uint32 >::max();


        std::vector< std::pair<point,std::string> > block;

        while(!is_shutdown && std::cin.good() && !std::cin.eof())
        {
            const point* input=istream.read();
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
            std::pair<point,std::string> input_pair = std::make_pair( *input, line );

            block.push_back(input_pair);
            last_block=input->block;
        }

        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-centroids: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-centroids: unknown exception" << std::endl; }
    return 1;
}
