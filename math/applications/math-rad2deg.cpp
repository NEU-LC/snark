// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <cmath>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <comma/string/string.h>

static void usage()
{
        std::cerr << std::endl;
        std::cerr << "convert angles in radians to degrees" << std::endl;
        std::cerr << std::endl;
        std::cerr << "usage: math-rad2deg <angle>,<angle>,..." << std::endl;
        std::cerr << "       math-rad2deg <angle> <angle> ..." << std::endl;
        std::cerr << std::endl;
        exit( -1 );
}

int main( int ac, char** av )
{
        if( ac == 1 ) { usage(); }
        bool firstSpace = true;
        std::cout.precision( 12 );
        long double ratio = ( long double )180 / M_PI;
        for( int i = 1; i < ac; ++i )
        {
                if( !firstSpace ) { std::cout << " "; } else { firstSpace = false; }
                const std::vector < std::string >& v = comma::split( av[i], ',' );
                bool firstComma = true;
                for( unsigned int k = 0; k < v.size(); ++k )
                {
                        double angle = boost::lexical_cast< double >( v[k] );
                        if( !firstComma ) { std::cout << ","; } else { firstComma = false; }
                        std::cout << ( ratio * angle );
                }
        }
        return 0;
}
