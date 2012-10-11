#ifndef SNARK_SENSORS_VELODYNE_THIN_REGION
#define SNARK_SENSORS_VELODYNE_THIN_REGION

#include <comma/math/cyclic.h>
#include <comma/visiting/traits.h>

namespace snark {  namespace velodyne { namespace thin {

/// region, quick and dirty
struct region
{
    virtual ~region() {}
    virtual bool has( double range, double bearing, double elevation ) const = 0;
    virtual double coverage() const = 0;
};

/// sector, quick and dirty
struct sector : public region
{
    sector();
    sector( double bearing, double ken, double range = 0 );
    bool has( double range, double bearing, double ) const;
    double coverage() const;
    comma::math::cyclic< double > bearing;
    double ken;
    double range;
};

} } } // namespace snark {  namespace velodyne { namespace thin {

namespace comma { namespace visiting { // quick and dirty

template <> struct traits< snark::velodyne::thin::sector >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::velodyne::thin::sector& p, Visitor& v )
    {
        v.apply( "bearing", p.bearing() );
        v.apply( "ken", p.ken );
        v.apply( "range", p.range );
    }

    template < typename Key, class Visitor >
    static void visit( const Key&, snark::velodyne::thin::sector& p, Visitor& v )
    {
        double bearing = p.bearing();
        v.apply( "bearing", bearing );
        p.bearing = bearing;
        //p.bearing = comma::math::cyclic< double >( snark::math::Interval< double >( -180.0, 180.0 ), bearing );
        v.apply( "ken", p.ken );
        v.apply( "range", p.range );
    }
};

} } // namespace comma { namespace visiting {

#endif // #ifndev SNARK_SENSORS_VELODYNE_THIN_REGION

