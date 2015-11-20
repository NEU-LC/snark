#ifndef SNARK_MATH_SPHERICAL_GEOMETRY_GREAT_CIRCLE_H_
#define SNARK_MATH_SPHERICAL_GEOMETRY_GREAT_CIRCLE_H_

#include <set>
#include <vector>
#include <Eigen/Geometry>
#include <boost/optional.hpp>
#include <comma/math/compare.h>
#include "coordinates.h"

namespace snark { namespace spherical {

/// great circle arc (shortest of two possible)
/// a convenience class
/// @todo move to snark?
struct great_circle
{
    class arc
    {
        public:

            arc();

//             // back and forth conversions increases floating point error and unit test failure
// //             arc( const coordinates& begin, const coordinates& end ) : arc( begin.to_cartesian(), end.to_cartesian() ) {}
// //             arc( const  Eigen::Vector3d& begin, const coordinates& end ) : arc( begin, end.to_cartesian() ) {}
// //             arc( const coordinates& begin, const Eigen::Vector3d& end ) : arc( begin.to_cartesian(), end ) {}
//
            arc( const Eigen::Vector3d& begin, const Eigen::Vector3d& end );

            // conversion to/from cartesian is expensive all permutations should be implemented
            arc( const coordinates& begin, const coordinates& end );

            arc( const Eigen::Vector3d& begin, const coordinates& end );

            arc( const coordinates& begin, const Eigen::Vector3d& end );

            /// @param angle angular offset from begin in radians along the arc towards its end
            /// @return vector at that point
            Eigen::Vector3d at( double angle ) const;

            /// @return begin as normalized cartesian vector
            const Eigen::Vector3d& begin() const { return begin_; }

            /// @return end as normalized cartesian vector
            const Eigen::Vector3d& end() const { return end_; }

            /// @return begin as normalized cartesian vector
            const coordinates& begin_coordinates() const { return begin_coordinates_; }

            /// @return end as normalized cartesian vector
            const coordinates& end_coordinates() const { return end_coordinates_; }

            /// @return angle between begin and end
            double angle() const { return angle_axis_.angle(); }

            /// @return normal to begin-end plane as normalized cartesian vector
            const Eigen::Vector3d& axis() const { return angle_axis_.axis(); }

            /// @return normalized tangent at begin + angle as normalized cartesian vector
            Eigen::Vector3d tangent_at( double angle ) const;

            /// @return normalized tangent projection on the tangent plane in the north-east frame
            ///         at begin + angle as normalized cartesian vector
            Eigen::Vector2d tangent_in_navigation_frame_at( double angle ) const;

            /// @return bearing in the range [-pi, pi) at given angular offset from begin
            /// @note a convenience method, since tangent_in_navigation_frame_at()
            ///       most likely to be enough most of the time
            double bearing( double angle = 0 ) const;

            /// @return true, if the point belongs to the arc, including ends
            /// @note ATTENTION: currently, due to the accumulating computation error
            ///                  the precision is only up to approximately 0.02 degree
            bool has( const coordinates& p ) const;
            bool has( const Eigen::Vector3d& p ) const;

            /// @return true, if the point belongs to the arc, excluding ends
            bool has_inside( const coordinates& p ) const;
            bool has_inside( const Eigen::Vector3d& p ) const;

            /// return true, if two arcs belong to the same great circle and have common points
            bool overlaps( const arc& rhs ) const;

            /// @return an arc which is the shortest path (perpendicular) to this arc from a given point
            /// if perpendicular arc does not cross with this arc segment, path to the closest end is returned
            arc shortest_path( const coordinates &c ) const;

            /// @return the angle which this arc should rotate around its beginning point, such that its begin, end and the given coordinate c, locate on the same great_circle::arc
            double angle_towards( const coordinates &c ) const;

            operator std::string() const;

            /// quickly checks if the arcs may potentially intersect; faster than calling intersection_with directly
            bool may_intersect( const great_circle::arc& target ) const;

            /// TODO: add performance monitoring in the backlog
            /// @return intersection with an arc
            /// @note can be only one point, since we do not consider
            ///       arcs greater than pi
            boost::optional< coordinates > intersection_with( const arc& rhs ) const;
            
            static Eigen::AngleAxis< double > angle_axis(const Eigen::Vector3d &begin, const Eigen::Vector3d &end)
            { return Eigen::AngleAxis< double >( Eigen::Quaternion< double >::FromTwoVectors( begin, end ) ); }

            std::vector< coordinates > discretize( const boost::optional< double >& resolution, const boost::optional< unsigned int >& circle_size = boost::optional< unsigned int >() ) const;

        private:
            coordinates begin_coordinates_;
            coordinates end_coordinates_;
            Eigen::Vector3d begin_;
            Eigen::Vector3d end_;
            Eigen::AngleAxis< double > angle_axis_;
    };
};

} } // namespace snark { namespace spherical {

#endif // SNARK_MATH_SPHERICAL_GEOMETRY_GREAT_CIRCLE_H_
