// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#pragma once

#ifndef Q_MOC_RUN
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include "../../../visiting/eigen.h"
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#endif

namespace snark { namespace graphics { namespace view {

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

        CameraReader( const comma::csv::options& options );

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

} } } // namespace snark { namespace graphics { namespace view {

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::view::point_with_orientation >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::point_with_orientation& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::point_with_orientation& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }
};

} } // namespace comma { namespace visiting {
