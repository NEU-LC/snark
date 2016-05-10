#include <cmath>
#include <Eigen/Geometry>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "great_circle.h"
// #include <iomanip>

namespace snark { namespace spherical {

// quick and dirty
static bool equal_( const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs ) { return ( lhs - rhs ).lpNorm< Eigen::Infinity >() < coordinates::epsilon; }

// quick and dirty
static bool is_between_( const great_circle::arc& a, const Eigen::Vector3d& p ) // quick and dirty, faster than great_circle::arc::has()
{
    if( equal_( a.begin(), p ) || equal_( a.end(), p ) ) { return true; } // if( comma::math::equal( a.begin(), p, epsilon ) || comma::math::equal( a.end(), p, epsilon ) ) { return true; }
    double begin_angle = great_circle::arc::angle_axis(a.begin(), p ).angle();
    double end_angle = great_circle::arc::angle_axis(p, a.end() ).angle();
    return comma::math::equal( a.angle(), begin_angle + end_angle, coordinates::epsilon );
}

//qnd
template< typename T >
static inline bool is_between( const T &t, const T &d1, const T &d2 ) 
{
    return d1 < d2 ? ( t >= d1 && t <= d2 ) : ( t <= d1 && t >= d2 );
}

template< typename T >
static inline bool has_overlap( const T &t1, const T &t2, const T &d1, const T &d2 ) 
{
    return is_between( t1, d1, d2 )
        || is_between( t2, d1, d2 )
        || is_between( d1, t1, t2 )
        || is_between( d2, t1, t2 );
}

great_circle::arc::arc()
    : begin_coordinates_( 0, 0 )
    , end_coordinates_( 0, 0 )
    , begin_( 1, 0, 0 )
    , end_( 1, 0, 0 )
    , angle_axis_( 0, Eigen::Vector3d( 0, 1, 0 ) ) // quick and dirty
{ }

great_circle::arc::arc( const Eigen::Vector3d& begin, const Eigen::Vector3d& end )
    : begin_coordinates_( begin )
    , end_coordinates_( end )
    , begin_( begin )
    , end_( end )
    , angle_axis_( Eigen::Quaternion< double >::FromTwoVectors( begin_, end_ ) )
{
    if( comma::math::less( M_PI, angle() ) ) { COMMA_THROW( comma::exception, "only arcs less than pi supported; got " << angle() << " radian for begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
    //if( equal_(begin_, end_) ) { COMMA_THROW( comma::exception, "zero length great_circle::arc is not permitted: got  begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
}

// conversion to/from cartesian is expensive all permutations should be implemented
great_circle::arc::arc( const coordinates& begin, const coordinates& end )
    : begin_coordinates_( begin )
    , end_coordinates_( end )
    , begin_( begin.to_cartesian() )
    , end_( end.to_cartesian() )
    , angle_axis_( Eigen::Quaternion< double >::FromTwoVectors( begin_, end_ ) )
{
    if( comma::math::less( M_PI, angle() ) ) { COMMA_THROW( comma::exception, "only arcs less than pi supported; got " << angle() << " radian for begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
    //if( equal_(begin_, end_) ) { COMMA_THROW( comma::exception, "zero length great_circle::arc is not permitted: got  begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
}

great_circle::arc::arc( const Eigen::Vector3d& begin, const coordinates& end )
    : begin_coordinates_( begin )
    , end_coordinates_( end )
    , begin_( begin )
    , end_( end.to_cartesian() )
    , angle_axis_( Eigen::Quaternion< double >::FromTwoVectors( begin_, end_ ) )
{
    if( comma::math::less( M_PI, angle() ) ) { COMMA_THROW( comma::exception, "only arcs less than pi supported; got " << angle() << " radian for begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
    //if( equal_(begin_, end_) ) { COMMA_THROW( comma::exception, "zero length great_circle::arc is not permitted: got  begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
}

great_circle::arc::arc( const coordinates& begin, const Eigen::Vector3d& end )
    : begin_coordinates_( begin )
    , end_coordinates_( end )
    , begin_( begin.to_cartesian() )
    , end_( end )
    , angle_axis_( Eigen::Quaternion< double >::FromTwoVectors( begin_, end_ ) )
{
    if( comma::math::less( M_PI, angle() ) ) { COMMA_THROW( comma::exception, "only arcs less than pi supported; got " << angle() << " radian for begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
    //if( equal_(begin_, end_) ) { COMMA_THROW( comma::exception, "zero length great_circle::arc is not permitted: got  begin: " << std::string(begin_coordinates_) << " end: " << std::string(end_coordinates_)); }
}


double great_circle::arc::bearing( double angle ) const
{
    Eigen::Vector2d t = tangent_in_navigation_frame_at( angle );
    double b = std::acos( t.x() ) * ( t.y() < 0 ? -1 : 1 );
    return b >= M_PI ? b - M_PI * 2 : b;
}

Eigen::Vector3d great_circle::arc::at( double angle ) const
{
    return Eigen::AngleAxis< double >( angle, angle_axis_.axis() ).toRotationMatrix() * begin_;
}

Eigen::Vector3d great_circle::arc::tangent_at( double angle ) const
{
    Eigen::Vector3d tangent = angle_axis_.axis().cross( at( angle ) );
    tangent.normalize();
    return tangent;
}

Eigen::Vector2d great_circle::arc::tangent_in_navigation_frame_at( double angle ) const
{
    const Eigen::Vector3d& v = at( angle );
    const Eigen::Vector3d& n = snark::spherical::to_navigation_frame( coordinates( v ), angle_axis_.axis().cross( v ) );
    Eigen::Vector2d projection( n.x(), n.y() ); //Eigen::Vector2d projection( v.y(), v.z() ); //Eigen::Vector2d projection( v.z(), v.y() );
    projection.normalize();
    return projection;
}

bool great_circle::arc::has_inside( const coordinates& p ) const
{ return has_inside(p.to_cartesian()); }

bool great_circle::arc::has_inside( const Eigen::Vector3d& p ) const
{
    //if( begin_coordinates_ == p || end_coordinates_ == p ) { return false; }
    if( equal_(begin_, p) || equal_(end_, p) ) { return false; }

    //great_circle::arc::angle_axis(begin_, p );
    Eigen::AngleAxis< double > a = great_circle::arc::angle_axis(begin_, p );

    return equal_( axis(), a.axis() ) && comma::math::less( a.angle(), angle() );
}

bool great_circle::arc::overlaps( const great_circle::arc& rhs ) const // quick and dirty, potentially slow
{
    return    ( equal_( axis(), rhs.axis() ) || equal_( axis(), -rhs.axis() ) )
           && (    has( rhs.begin_ )
                || has( rhs.end_ )
                || rhs.has( begin_ )
                || rhs.has( end_ ) );
//     return    ( equal_( axis(), rhs.axis() ) || equal_( axis(), -rhs.axis() ) )
//            && (    has( rhs.begin_coordinates() )
//                 || has( rhs.end_coordinates() )
//                 || rhs.has( begin_coordinates_ )
//                 || rhs.has( end_coordinates_ ) );
}

great_circle::arc great_circle::arc::shortest_path( const coordinates &c ) const
{
    Eigen::Vector3d cc(c.to_cartesian());
    if (angle() == 0) { return arc(cc, begin()); }
    if (has_inside(c)) { return arc(cc, cc); }

    Eigen::Vector3d n = end() - begin();
    Eigen::Vector3d p((n*(n.dot(cc - begin())))/n.dot(n) + begin());
    boost::optional< coordinates > t = intersection_with(arc(p, cc));
    if (! t)
    {
        arc a1(cc, begin());
        arc a2(cc, end());
        return a1.angle() < a2.angle() ? a1 : a2;
    }
    return arc(cc, *t);
}

double great_circle::arc::angle_towards( const coordinates &c ) const
{
    if( has(c) ) { return 0; }

    // todo: using bearing may be very inaccurate; simply turning the arc axis should be the right way, literally something like this:
    // Eigen::Quaternion< double >::FromTwoVectors( axis(), great_circle::arc( begin_coordinates_, c ).axis() ).angle()

    double result = bearing(0) - arc(begin_coordinates_, c).bearing(0);
    if (result > M_PI) { result = -2*M_PI + result; }
    return result;
}

great_circle::arc::operator std::string() const
{
    return "[" + std::string(begin_coordinates()) + " -> " +std::string(end_coordinates()) + "]";
}

// struct count_t
// {
//     unsigned int accepted;
//     unsigned int rejected;
//     count_t() : accepted( 0 ), rejected( 0 ) {}
//     //~count_t() { std::cerr << "=============> great_circle::arc::may_intersect: accepted: " << accepted << " rejected: " << rejected << std::endl; }
// };
//
// static count_t count;

static const double two_pi = M_PI * 2;
bool great_circle::arc::may_intersect(const great_circle::arc& target) const
{
    double self_begin_longitude = begin_coordinates_.longitude;
    double self_end_longitude = end_coordinates_.longitude;
    double other_begin_longitude = target.begin_coordinates_.longitude;
    double other_end_longitude = target.end_coordinates_.longitude;
    // if either self or target wraps longitude over 180, unwrap
    bool wrap_self = false;
    if ( std::abs( self_end_longitude  - self_begin_longitude  ) > M_PI ) { self_end_longitude  += two_pi; wrap_self  = true; }
    bool wrap_other = false;
    if ( std::abs( other_end_longitude - other_begin_longitude ) > M_PI ) { other_end_longitude += two_pi; wrap_other = true; }
    // double conv = 180.0/M_PI;
    // std::cerr << std::setprecision(12) << "self:  [" << begin_coordinates_.longitude*conv        << "," << end_coordinates_.longitude*conv        << "], [" << self_begin_longitude*conv  << "," << self_end_longitude*conv  << "], " << wrap_self << std::endl;
    // std::cerr << std::setprecision(12) << "other: [" << target.begin_coordinates_.longitude*conv << "," << target.end_coordinates_.longitude*conv << "], [" << other_begin_longitude*conv << "," << other_end_longitude*conv << "], " << wrap_self << std::endl;
    // now both intervals are unwrapped; the may overlap directly or with a shift
    if (               has_overlap( other_begin_longitude,          other_end_longitude,          self_begin_longitude,          self_end_longitude          ) ) return true;
    if ( wrap_self  && has_overlap( other_begin_longitude + two_pi, other_end_longitude + two_pi, self_begin_longitude,          self_end_longitude          ) ) return true;
    if ( wrap_other && has_overlap( other_begin_longitude,          other_end_longitude,          self_begin_longitude + two_pi, self_end_longitude + two_pi ) ) return true;
    return false;
}

bool great_circle::arc::has( const coordinates& p ) const
{ return has(p.to_cartesian()); }

bool great_circle::arc::has( const Eigen::Vector3d& p ) const
{
    //if( begin_coordinates_ == p || end_coordinates_ == p ) { return true; }
    if( equal_(begin_, p) || equal_(end_, p )) { return true; }

    //great_circle::arc a( begin_coordinates_, p );
    Eigen::AngleAxis< double > a = arc::angle_axis(begin_, p);

    return equal_( axis(), a.axis() ) && comma::math::less( a.angle(), angle() );
}

// /// @return intersection with an arc
// /// @note can be only one point, since we do not consider
// ///       arcs greater than pi
// boost::optional< coordinates > great_circle::arc::intersection_with( const arc& rhs ) const
// {
//     if (!may_intersect(rhs))  { return boost::optional< coordinates >(); }
//     if( equal_( axis(), rhs.axis() ) || equal_( axis(), -rhs.axis() ) ) { return boost::optional< coordinates >(); }
//     Eigen::Vector3d cross = axis().cross( rhs.axis() );
//     snark::range_bearing_elevation a( cross ); // quick and dirty, quite excessive
//     coordinates c( a.e(), a.b() );
//     if( has( c ) && rhs.has( c ) ) { return c; }
//     snark::range_bearing_elevation b( -cross ); // quick and dirty, quite excessive
//     coordinates d( b.e(), b.b() );
//     if( has( d ) && rhs.has( d ) ) { return d; }
//     return boost::optional< coordinates >();
// }

boost::optional< coordinates > great_circle::arc::intersection_with( const arc& rhs ) const
{
    if ( !may_intersect(rhs) )  {
        // std::cerr << "cannot intersect" << std::endl;
        return boost::optional< coordinates >();
    }
    if( equal_( axis(), rhs.axis() ) || equal_( axis(), -rhs.axis() ) ) {
        // std::cerr << "are equal" << std::endl;
        return boost::optional< coordinates >();
    }
    Eigen::Vector3d cross = axis().cross( rhs.axis() );
    if( is_between_( *this, cross ) && is_between_( rhs, cross ) )
    {
        snark::range_bearing_elevation a( cross ); // quick and dirty, quite excessive
        return coordinates( a.e(), a.b() );
    }
    if( is_between_( *this, -cross ) && is_between_( rhs, -cross ) )
    {
        snark::range_bearing_elevation a( -cross ); // quick and dirty, quite excessive
        return coordinates( a.e(), a.b() );
    }
    return boost::optional< coordinates >();
}

std::vector< coordinates > great_circle::arc::discretize( const boost::optional< double >& resolution, const boost::optional< unsigned int >& circle_size ) const
{
    boost::optional< double > r = resolution;
    boost::optional< double > s;
    if( circle_size ) { s = M_PI * 2 / *circle_size; }
    if( ( !r && s ) || ( s && *r > *s ) ) { r = s; }
    if( !r ) { COMMA_THROW( comma::exception, "expected either resolution or circle size, got none" ); }
    std::vector< coordinates > v;
    v.reserve( angle() / *r + 1 );
    for( double a = 0; a < angle(); v.push_back( coordinates( at( a ) ) ), a += *r );
    if( v.back() != end_coordinates_ ) { v.push_back( end_coordinates_ ); }
    return v;
}

} } // namespace snark { namespace spherical {
