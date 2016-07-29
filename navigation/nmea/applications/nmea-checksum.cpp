// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
