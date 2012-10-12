#ifndef SNARK_SENSORS_VELODYNE_VELODYNESTREAM_H_
#define SNARK_SENSORS_VELODYNE_VELODYNESTREAM_H_

#ifndef WIN32
#include <stdlib.h>
#endif
#include <snark/sensors/velodyne/stream.h>
#include <snark/visiting/eigen.h>

namespace snark {

/// processed velodyne point    
struct velodyne_point
{
    boost::posix_time::ptime timestamp;
    comma::uint32 id;
    comma::uint32 intensity;
    std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray;
    double azimuth;
    double range;
    bool valid;
    comma::uint32 scan;
};

/// convert stream of raw velodyne data into velodyne points
template < typename S >
class velodyne_stream
{   
public:    
    velodyne_stream( const velodyne::db& db
                  , bool outputInvalidpoints
                  , boost::optional< std::size_t > from = boost::optional< std::size_t >(), boost::optional< std::size_t > to = boost::optional< std::size_t >() );

    template < typename P >
    velodyne_stream( const P& p
                  , const velodyne::db& db
                  , bool outputInvalidpoints
                  , boost::optional< std::size_t > from = boost::optional< std::size_t >(), boost::optional< std::size_t > to = boost::optional< std::size_t >() );

    bool read();
    const velodyne_point& point() const { return m_point; }

private:
    velodyne::stream< S > m_stream;
    velodyne::db m_db;
    velodyne_point m_point;
    boost::optional< std::size_t > m_to;
};

template < typename S >
velodyne_stream< S >::velodyne_stream ( const velodyne::db& db, bool outputInvalidpoints
                    , boost::optional< std::size_t > from
                    , boost::optional< std::size_t > to ):
    m_stream( new S, outputInvalidpoints ),
    m_db( db ),
    m_to( to )
{
    if( from ) { while( m_stream.scan() < *from ) { m_stream.skipscan(); } }
}

template < typename S >
template < typename P >
velodyne_stream< S >::velodyne_stream ( const P& p, const velodyne::db& db, bool outputInvalidpoints
                    , boost::optional< std::size_t > from
                    , boost::optional< std::size_t > to ):
    m_stream( new S( p ), outputInvalidpoints ),
    m_db( db ),
    m_to( to )
{
    if( from ) { while( m_stream.scan() < *from ) { m_stream.skipscan(); } }
}

/// read and convert one point from the stream
/// @return false if end of stream is reached
template < typename S >
bool velodyne_stream< S >::read()
{
    if( m_to && m_stream.scan() > *m_to )
    {
        return false;
    }

    const velodyne::laser_return* r = m_stream.read();

    if( r == NULL ) { return false; }
    m_point.timestamp = r->timestamp;
    m_point.id = r->id;
    m_point.intensity = r->intensity;
    m_point.valid = !comma::math::equal( r->range, 0 ); // quick and dirty
    m_point.ray = m_db.lasers[ m_point.id ].ray( r->range, r->azimuth );
    m_point.range = m_db.lasers[ m_point.id ].range( r->range );
    m_point.scan = m_stream.scan();
    m_point.azimuth = m_db.lasers[ m_point.id ].azimuth( r->azimuth );
    return true;
}

/// specialisation for csv input stream: in this case nothing to convert
template <>
class velodyne_stream< comma::csv::input_stream< velodyne_point> >
{
public:

    velodyne_stream( const comma::csv::options& options ): m_stream( std::cin, options ) {}

    bool read();
    const velodyne_point& point() const { return m_point; }

private:
    comma::csv::input_stream< velodyne_point > m_stream;
    velodyne_point m_point;
};

/// read and forward one point from the stream: nothing to convert as the input is csv
/// @return false if end of stream is reached
inline bool velodyne_stream< comma::csv::input_stream< velodyne_point> >::read()
{
    const velodyne_point* point = m_stream.read();
    if( point == NULL )
    {
        return false;
    }
    m_point = *point;
    return true;
}


} 


namespace comma { namespace visiting {

template <> struct traits< snark::velodyne_point >
{    
    template < typename K, typename V > static void visit( const K&, snark::velodyne_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "id", p.id );
        v.apply( "intensity", p.intensity );
        v.apply( "ray", p.ray );
        v.apply( "azimuth", p.azimuth );
        v.apply( "range", p.range );
        v.apply( "valid", p.valid );
        v.apply( "scan", p.scan );
    }
    
    template < typename K, typename V > static void visit( const K&, const snark::velodyne_point& p, V& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "id", p.id );
        v.apply( "intensity", p.intensity );
        v.apply( "ray", p.ray );
        v.apply( "azimuth", p.azimuth );
        v.apply( "range", p.range );
        v.apply( "valid", p.valid );
        v.apply( "scan", p.scan );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_SENSORS_VELODYNE_VELODYNESTREAM_H_
