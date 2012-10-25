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

#ifndef SNARK_POINTS_FRAME_HEADER
#define SNARK_POINTS_FRAME_HEADER

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/optional.hpp>

#include <Eigen/Geometry>
#include <comma/base/exception.h>
#include <comma/csv/options.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <snark/math/rotation_matrix.h>
#include <snark/visiting/eigen.h>
#include "./timestamped.h"

namespace snark{ namespace applications {

struct position
{
    position() : coordinates( Eigen::Vector3d::Zero() ), orientation( Eigen::Vector3d::Zero() ) {}
    position( const Eigen::Vector3d& p ) : coordinates( p ), orientation( Eigen::Vector3d::Zero() ) {}
    position( const Eigen::Vector3d& p, const Eigen::Vector3d& o ) : coordinates( p ), orientation( o ) {}
    Eigen::Vector3d coordinates;
    Eigen::Vector3d orientation;
};

} }

namespace comma { namespace visiting {

template <> struct traits< snark::applications::position >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, snark::applications::position& p, Visitor& v )
    {
        v.apply( "x", p.coordinates.x() );
        v.apply( "y", p.coordinates.y() );
        v.apply( "z", p.coordinates.z() );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::applications::position& p, Visitor& v )
    {
        v.apply( "x", p.coordinates.x() );
        v.apply( "y", p.coordinates.y() );
        v.apply( "z", p.coordinates.z() );
        v.apply( "roll", p.orientation.x() );
        v.apply( "pitch", p.orientation.y() );
        v.apply( "yaw", p.orientation.z() );
    }
};

} }

namespace snark{ namespace applications {

class frame
{
    public:
        typedef timestamped< position, boost::posix_time::ptime > position_type;
        
        typedef position_type point_type;

        frame( const position& p, bool to = false, bool interpolate = true, bool rotation_present = false );

        frame( const comma::csv::options& options, bool discardOutOfOrder, boost::optional< boost::posix_time::time_duration > maxGap, bool outputframe, bool to = false, bool interpolate = true, bool rotation_present = false );

        ~frame();

        const point_type* converted( const point_type& rhs );

        bool discarded() const { return m_discarded; }

        const position& last() const { return m_position; }

        const bool outputframe;

    private:
        void set_position( const position& p );

        const point_type& convert( const point_type& rhs );

        bool m_to; /// to frame if true, from frame else
        bool m_interpolate; /// interpolate nav data
        position m_position;
        ::Eigen::Vector3d m_orientation; /// roll pitch yaw
        ::Eigen::Matrix3d m_rotation; ///rotation
        ::Eigen::Affine3d m_transform; /// fromframe or toframe transform
        ::Eigen::Translation3d m_translation; /// translation

        boost::scoped_ptr< comma::io::istream > m_istream;
        boost::scoped_ptr< comma::csv::input_stream< position_type > > m_is;
        std::pair< position_type, position_type > m_pair; /// nav pair
        point_type m_converted;
        bool m_discardOutOfOrder;
        bool m_discarded;
        boost::optional< boost::posix_time::time_duration > m_maxGap;
        bool rotation_present_;
};

} } // namespace snark{ namespace applications {

#endif // SNARK_POINTS_FRAME_HEADER
