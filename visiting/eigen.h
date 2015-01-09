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
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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


#ifndef SNARK_VISITING_EIGEN_H
#define SNARK_VISITING_EIGEN_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <comma/visiting/apply.h>
#include <comma/visiting/visit.h>

namespace comma { namespace visiting {

template < typename T > struct traits< ::Eigen::Matrix< T, 2, 1 > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ::Eigen::Matrix< T, 2, 1 >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ::Eigen::Matrix< T, 2, 1 >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
    }
};

template < typename T > struct traits< ::Eigen::Matrix< T, 3, 1 > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ::Eigen::Matrix< T, 3, 1 >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ::Eigen::Matrix< T, 3, 1 >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }
};

template < typename T > struct traits< ::Eigen::Matrix< T, 4, 1 > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ::Eigen::Matrix< T, 4, 1 >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
        v.apply( "w", p.w() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ::Eigen::Matrix< T, 4, 1 >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
        v.apply( "w", p.w() );
    }
};


template < typename T > struct traits< ::Eigen::Quaternion< T > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ::Eigen::Quaternion< T >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
        v.apply( "w", p.w() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ::Eigen::Quaternion< T >& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
        v.apply( "w", p.w() );
    }
};

template < typename T > struct traits< ::Eigen::Translation< T, 3 > >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, ::Eigen::Translation< T, 3 >& p, Visitor& v )
    {
        v.apply( "x", p.vector().x() );
        v.apply( "y", p.vector().y() );
        v.apply( "z", p.vector().z() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const ::Eigen::Translation< T, 3 >& p, Visitor& v )
    {
        v.apply( "x", p.vector().x() );
        v.apply( "y", p.vector().y() );
        v.apply( "z", p.vector().z() );
    }
};


} }

#endif // SNARK_VISITING_EIGEN_H

