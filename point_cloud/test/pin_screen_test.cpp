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


#include <set>
#include <gtest/gtest.h>
#include "../impl/pin_screen.h"

namespace snark { namespace Robotics {

typedef pin_screen< int >::index_type index_type;
typedef pin_screen< int >::size_type size_type;

TEST( pin_screen, grid )
{
    pin_screen< int > grid( 10, 12 );
    grid( 2, 3, 4 ) = 5;
    EXPECT_EQ( grid.size(), ( size_type( 10, 12 ) ) );
    EXPECT_EQ( grid( 2, 3, 4 ), 5 );
    EXPECT_TRUE( grid.exists( 2, 3, 4 ) );
    EXPECT_EQ( grid.column( 2, 3 ).size(), 1u );
    EXPECT_EQ( grid.height( 2, 3 ), 4u );
    grid( 2, 3, 10 ) = 6;
    EXPECT_TRUE( grid.exists( 2, 3, 10 ) );
    EXPECT_EQ( grid.column( 2, 3 ).size(), 2u );
    EXPECT_EQ( grid.height( 2, 3 ), 10u );
    grid.erase( 2, 3, 4 );
    grid.erase( 2, 3, 10 );
    EXPECT_TRUE( !grid.exists( 2, 3, 4 ) );
    EXPECT_EQ( grid.column( 2, 3 ).size(), 0u );
    EXPECT_EQ( grid.height( 2, 3 ), 0u );
}

TEST( pin_screen, copy )
{
    {
        pin_screen< int > p( 4, 4 );
        p( 1, 2, 3 ) = 5;
        pin_screen< int > q( p );
        EXPECT_EQ( q( 1, 2, 3 ), 5 );
        q = p;
        EXPECT_EQ( q( 1, 2, 3 ), 5 );
    }
}

static const unsigned int size( 3 );

template < typename It, typename S >
static void Testpin_screeniterator( S& shape )
{
//     std::cerr << std::endl << "Testpin_screeniterator" << std::endl;
//     for( unsigned int i = 0; i < size; ++i )
//     {
//         for( unsigned int j = 0; j < size; ++j )
//         {
//             std::cerr << shape[i][j] << " ";
//         }
//         std::cerr << std::endl;
//     }
//     std::cerr << "----------------" << std::endl;
//     std::cerr << std::endl;
    {
        unsigned int count( 0 );
        pin_screen< int > pinscreen( size, size );
        for( unsigned int i = 0; i < size; ++i )
        {
            for( unsigned int j = 0; j < size; ++j )
            {
                count += shape[i][j];
                for( int k = 0; k < shape[i][j]; ++k )
                {
                    pinscreen( i, j, k ) = k;
                    EXPECT_TRUE( pinscreen.exists( i, j, k ) );
                    EXPECT_EQ( pinscreen( i, j, k ), k );
                    EXPECT_EQ( pinscreen.column( i, j ).size(), 1u + k );
                    EXPECT_EQ( pinscreen.height( i, j ), 0u + k );
                }
            }
        }
        std::set< It > s;
        for( It it = pinscreen.begin(); it != pinscreen.end(); ++it )
        {
            EXPECT_TRUE( pinscreen.exists( it() ) );
            s.insert( it );
        }
        EXPECT_TRUE( s.size() == count );
    }
    {
        unsigned int count( 0 );
        pin_screen< int > pinscreen( size, size );
        for( unsigned int i = 0; i < size; ++i )
        {
            for( unsigned int j = 0; j < size; ++j )
            {
                if( shape[i][j] == 0 ) { continue; }
                int k( shape[i][j] - 1 );
                ++count;
                pinscreen( i, j, k ) = k;
                EXPECT_TRUE( pinscreen.exists( i, j, k ) );
                EXPECT_EQ( pinscreen( i, j, k ), k );
                EXPECT_EQ( pinscreen.column( i, j ).size(), 1u );
                EXPECT_EQ( pinscreen.height( i, j ), 0u + k );
            }
        }
        std::set< It > s;
        for( It it = pinscreen.begin(); it != pinscreen.end(); ++it )
        {
            EXPECT_TRUE( pinscreen.exists( it() ) );
            s.insert( it );
        }
        EXPECT_TRUE( s.size() == count );
    }
}

template < typename S >
static void Testpin_screenneighbourhood_iterator( S& shape, S& sizes )
{
//    for( unsigned int i = 0; i < size; ++i )
//    {
//        for( unsigned int j = 0; j < size; ++j )
//        {
//            std::cerr << shape[i][j] << " ";
//        }
//        std::cerr << std::endl;
//    }
//    std::cerr << std::endl;
    {
        unsigned int count( 0 );
        pin_screen< int > pinscreen( size, size );
        for( unsigned int i = 0; i < size; ++i )
        {
            for( unsigned int j = 0; j < size; ++j )
            {
                if( shape[i][j] == 0 ) { continue; }
                int k( shape[i][j] - 1 );
                ++count;
                pinscreen( i, j, k ) = k;
                EXPECT_TRUE( pinscreen.exists( i, j, k ) );
                EXPECT_EQ( pinscreen( i, j, k ), k );
                EXPECT_EQ( pinscreen.column( i, j ).size(), 1u );
                EXPECT_EQ( pinscreen.height( i, j ), 0u + k );
            }
        }
        for( pin_screen< int >::iterator it = pinscreen.begin(); it != pinscreen.end(); ++it )
        {
            //TEST_PRINT( it() );
            EXPECT_TRUE( pinscreen.exists( it() ) );
            pin_screen< int >::neighbourhood_iterator begin( pin_screen< int >::neighbourhood_iterator::begin( it ) );
            pin_screen< int >::neighbourhood_iterator end( pin_screen< int >::neighbourhood_iterator::end( it ) );
            std::set< pin_screen< int >::iterator > s;
            for( pin_screen< int >::neighbourhood_iterator nit( begin ); nit != end; ++nit )
            {
                //TEST_PRINT( nit() );
                s.insert( nit );
            }
            EXPECT_EQ( s.size(), sizes[it()[0]][it()[1]] );
        }
    }
}


TEST( pin_screen, Largepin_screenneighbourhood_iterator )
{
    static const unsigned int sizeLarge( 10 );
    {
        pin_screen< int > pinscreen( sizeLarge , sizeLarge );
        unsigned int check[sizeLarge][sizeLarge];
        for( unsigned int i = 0; i < sizeLarge ; ++i )
        {
            for( unsigned int j = 0; j < sizeLarge; ++j )
            {
                check[i][j] = 0;
                if( i == j ) { pinscreen( i, j , 0 ) = i; }
            }
        }
        for( pin_screen< int >::iterator it = pinscreen.begin(); it != pinscreen.end(); ++it )
        {
            EXPECT_EQ( check[it()[0]][it()[1]] , 0u );
            check[it()[0]][it()[1]] = 1;
            if( it()[0] == it()[1] )
            {
                EXPECT_TRUE( pinscreen.exists( it()[0], it()[1], it()[2] ) );
                EXPECT_EQ( it()[2] , 0u );
                EXPECT_EQ( static_cast<unsigned int>( pinscreen( it() ) ) , it()[0] );
                pin_screen< int >::neighbourhood_iterator begin( pin_screen< int >::neighbourhood_iterator::begin( it ) );
                pin_screen< int >::neighbourhood_iterator end( pin_screen< int >::neighbourhood_iterator::end( it ) );
                std::set< pin_screen< int >::iterator > s;
                for( pin_screen< int >::neighbourhood_iterator nit( begin ); nit != end; ++nit )
                {
                    s.insert( nit );
                    if( nit()[0] == nit()[1] && nit()[0] > 0 && nit()[0] < sizeLarge-1 )
                    {
                        pin_screen< int >::neighbourhood_iterator begin2( pin_screen< int >::neighbourhood_iterator::begin( nit ) );
                        pin_screen< int >::neighbourhood_iterator end2( pin_screen< int >::neighbourhood_iterator::end( nit ) );
                        std::set< pin_screen< int >::iterator > s2;
                        for( pin_screen< int >::neighbourhood_iterator nit2( begin2 ); nit2 != end2; ++nit2 )
                        {
                            s2.insert( nit2 );
                        }
                        EXPECT_EQ( s2.size() , 2u );
                    }
                }

                if( it()[0] == 0 || it()[0] == sizeLarge-1 )
                {
                    EXPECT_EQ( s.size() , 1u );
                } else {
                    EXPECT_EQ( s.size() , 2u );
                }
            }
        }
    }
}

template < typename It >
static void Testpin_screeniterator()
{
    {
        const int shape[size][size] = { { 0, 0, 0 }
                                      , { 0, 0, 0 }
                                      , { 0, 0, 0 } };
        Testpin_screeniterator< It >( shape );
    }
    {
        const int shape[size][size] = { { 0, 0, 0 }
                                      , { 0, 0, 0 }
                                      , { 0, 0, 1 } };
        Testpin_screeniterator< It >( shape );
    }
    {
        const int shape[size][size] = { { 0, 5, 0 }
                                      , { 0, 6, 0 }
                                      , { 0, 0, 1 } };
        Testpin_screeniterator< It >( shape );
    }
    {
        const int shape[size][size] = { { 1, 1, 1 }
                                      , { 1, 1, 1 }
                                      , { 1, 1, 1 } };
        Testpin_screeniterator< It >( shape );
    }
    {
        const int shape[size][size] = { { 2, 2, 2 }
                                      , { 2, 2, 2 }
                                      , { 2, 2, 2 } };
        Testpin_screeniterator< It >( shape );
    }
}

TEST( Pinscreen, neighbourhood_iterator )
{
    {
        const unsigned int shape[size][size] = { { 0, 0, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        for( unsigned int i = 0; i < size; ++i )
        {
            for( unsigned int j = 0; j < size; ++j )
            {
                unsigned int shape[size][size] = { { 0, 0, 0 }
                                                 , { 0, 0, 0 }
                                                 , { 0, 0, 0 } };
                unsigned int sizes[size][size] = { { 0, 0, 0 }
                                                 , { 0, 0, 0 }
                                                 , { 0, 0, 0 } };
                shape[i][j] = 1;
                Testpin_screenneighbourhood_iterator( shape, sizes );
                shape[i][j] = 2;
                Testpin_screenneighbourhood_iterator( shape, sizes );
            }
        }
    }
    {
        const unsigned int shape[size][size] = { { 1, 1, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 1, 1, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 1, 2, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 1, 1, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 1, 3, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 2, 3, 0 }
                                               , { 1, 2, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 3, 2, 0 }
                                               , { 2, 3, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 0, 0, 0 }
                                               , { 1, 1, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 1, 1, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 0, 0, 0 }
                                               , { 1, 2, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 1, 1, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 0, 0, 0 }
                                               , { 1, 3, 0 }
                                               , { 0, 0, 0 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 0, 0, 0 }
                                               , { 0, 0, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 0, 0, 0 }
                                               , { 2, 3, 0 }
                                               , { 1, 2, 0 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 3, 2, 0 }
                                               , { 2, 3, 0 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    {
        const unsigned int shape[size][size] = { { 0, 0, 0 }
                                               , { 0, 2, 3 }
                                               , { 0, 1, 2 } };
        const unsigned int sizes[size][size] = { { 0, 0, 0 }
                                               , { 0, 3, 2 }
                                               , { 0, 2, 3 } };
        Testpin_screenneighbourhood_iterator( shape, sizes );
    }
    // TODO: more unit tests
}


TEST( pin_screen, iterators )
{
    Testpin_screeniterator< pin_screen< int >::const_iterator >();
    Testpin_screeniterator< pin_screen< int >::iterator >();
}


} } // namespace snark { namespace Robotics {

