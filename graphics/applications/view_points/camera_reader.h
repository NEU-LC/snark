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


/// @author Vsevolod Vlaskine, Cedric Wohlleber

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_CAMERAREADER_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_CAMERAREADER_H_

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <snark/visiting/eigen.h>

namespace snark { namespace graphics { namespace View {

struct point_with_orientation // quick and dirty
{
    Eigen::Vector3d point;
    Eigen::Vector3d orientation;
    point_with_orientation() : point( 0, 0, 0 ), orientation( 0, 0, 0 ) {}
    bool operator==( const point_with_orientation& rhs ) const { return point == rhs.point && orientation == rhs.orientation; }
    bool operator!=( const point_with_orientation& rhs ) const { return !operator==( rhs ); }
};

class CameraReader
{
    public:
        const comma::csv::options options;

        CameraReader( comma::csv::options& options );

        void start();
        bool read_once();
        bool ready() const { return m_stream->ready(); }
        void render();

        bool isShutdown() const;
        void shutdown();
        Eigen::Vector3d position() const;
        Eigen::Vector3d orientation() const;
        void read();

    private:
        bool m_shutdown;
        comma::io::istream m_istream;
        Eigen::Vector3d m_position;
        Eigen::Vector3d m_orientation;
        boost::scoped_ptr< comma::csv::input_stream< point_with_orientation > > m_stream;
        mutable boost::recursive_mutex m_mutex; // todo: make lock-free
        boost::scoped_ptr< boost::thread > m_thread;

};

} } } // namespace snark { namespace graphics { namespace View {

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::View::point_with_orientation >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::View::point_with_orientation& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::View::point_with_orientation& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }
};

} } // namespace comma { namespace visiting {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_CAMERAREADER_H_*/
