#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <iostream>
#include <string>
#include "../dualem.h"

void usage(bool detail)
{
    std::cerr << "    read raw dualem message data from stdin and output formated records to stdout " << std::endl;
    std::cerr << "        messages are in NMEA format " << std::endl;
    std::cerr << std::endl;
    std::cerr << "example: " << std::endl;
    std::cerr << "    socat -u tcp:shrimp.littleboard:50099 - | dualem-to-csv " << std::endl;
    std::cerr << std::endl;
    exit(0);
}

int main( int ac, char** av )
{
    std::cerr<<"not implemented"<<std::endl; exit(1);
    comma::command_line_options options( ac, av, usage );
    try
    {
        snark::dualem::dualem dualem;
        while(std::cin.good())
        {
            std::string msg;
            std::getline(std::cin, msg);
            std::cerr<<"msg "<<msg<<std::endl;
            if(!dualem.process_message(msg))
                std::cerr<<"process failed"<<std::endl;
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; return 1;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; return 1;
    }
    return 0;
}
