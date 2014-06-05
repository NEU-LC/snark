#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | csv-to-kml [<what>] [<options>] > points.kml" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << std::endl;
}

struct coordinates
{
    double latitude;
    double longitude;
};

// todo: traits

int main( int ac, char** av )
{
    comma::command_line_options options( ac, av, usage );

    std::cerr << "csv-to-kml: todo" << std::endl;

    exit( 1 );
}
