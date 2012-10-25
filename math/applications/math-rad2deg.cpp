// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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
