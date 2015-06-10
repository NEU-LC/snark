

#include "../dem/srtm/header.h"
#include "../dem/srtm/load.h"

int main( int ac, char** av )
{
    snark::terrain::dem::srtm::header h;
    load( h, av[1] );
}
