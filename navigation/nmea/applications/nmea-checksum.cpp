#include <iomanip>
#include <iostream>
#include <comma/application/command_line_options.h>
#include "../string.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "read nmea sentence and calculate checksum" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: echo <nmea-sentence> | nmea-checksum" << std::endl;
    std::cerr << std::endl;
    std::cerr << "if start of sequence '$' and/or checksum delimiter '*' are present," << std::endl;
    std::cerr << "only the characters between will be used." << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: output help" << std::endl;
    std::cerr << std::endl;
}

int main( int ac, char **av )
{
    comma::command_line_options options( ac, av, usage );
    while( std::cin.good() )
    {
        std::string line;
        std::getline( std::cin, line );
        if( line.empty() ) { continue; }
        std::cout << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << (int)snark::nmea::string::checksum( line ) << std::endl;
    }
    return 0;
}
