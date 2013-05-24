#ifndef SNARK_POINT_CLOUD_SPHERICAL_GRID_H_
#define SNARK_POINT_CLOUD_SPHERICAL_GRID_H_

#include <boost/multi_array.hpp>
#include <snark/math/range_bearing_elevation.h>

namespace snark {
    
/// a simple 2d bearing-elevation grid
struct bearing_elevation_grid
{
    /// index
    class index
    {
        public:
            /// index type
            typedef boost::array< std::size_t, 2 > type;

            /// constructors
            index() {}
            index( double bearing_begin, double elevation_begin, double bearing_resolution, double elevation_resolution );
            index( double bearing_begin, double elevation_begin, double resolution );
            index( double bearing_resolution, double elevation_resolution );
            index( double resolution );
            index( const snark::bearing_elevation& begin, const snark::bearing_elevation& resolution );
            index( const snark::bearing_elevation& resolution );

            /// @return index relative to begin with given resolution
            type operator()( double bearing, double elevation ) const;

            /// @return index relative to begin with given resolution
            type operator()( const snark::bearing_elevation& v ) const;

            /// @return bearing, elevation for given index
            snark::bearing_elevation bearing_elevation( const type& i ) const;

            /// @return begin
            const snark::bearing_elevation& begin() const;

            /// @return resolution
            const snark::bearing_elevation& resolution() const;

        private:
            snark::bearing_elevation begin_;
            snark::bearing_elevation resolution_;
    };

    /// a convenience wrapper around boost::multi_array
    /// nothing prevents you from using other containers
    /// in conjunction with bearing_elevation_index
    template < typename T >
    class type : public boost::multi_array< T, 2 >
    {
        public:
            /// types
            typedef typename bearing_elevation_grid::index index_t;
            typedef boost::multi_array< T, 2 > base_t;

            /// constructors
            type( const index_t& i, const snark::bearing_elevation& end ) : base_t( boost::extents[ i( end )[0] ][ i( end )[1] ] ), index_( i ) {}
            type( const index_t& i, const typename index_t::type size ) : base_t( boost::extents[ size[0] ][ size[1] ] ), index_( i ) {}
            type( const index_t& i, std::size_t bearing_size, std::size_t elevation_size ) : base_t( boost::extents[ bearing_size ][ elevation_size ] ), index_( i ) {}

            /// accessors
            T& operator()( double bearing, double elevation ) { return this->base_t::operator()( index_( bearing, elevation ) ); }
            const T& operator()( double bearing, double elevation ) const { return this->base_t::operator()( index_( bearing, elevation ) ); }

            /// @return index
            const index_t& index() const { return index_; }

        private:
            index_t index_;
    };
    
    /// Required by bearing_index and elevation_index for interpolation support
    struct bounds
    {
        int lower_index;
        int upper_index;
        double weight;
    };

    /// bearing index; quick and dirty; required for some applications
    class bearing_index
    {
        public:
            typedef double key_t;
            /// constructors
            bearing_index() {}
            bearing_index( double begin, double resolution ) : index_( begin, 0, resolution, resolution ) {}
            bearing_index( double resolution ) : index_( resolution, resolution ) {}

            /// @return index relative to begin with given resolution
            std::size_t operator()( double bearing ) const { return index_( bearing, 0 )[0]; }

            /// @return begin
            double begin() const { return index_.begin().bearing(); }

            /// @return resolution
            double resolution() const { return index_.resolution().bearing(); }

            /// @return the lower and upper index between which value
            /// lies and corresponding weight from lower_index
            bounds get_bounds( const double value ) const;

        private:
            bearing_elevation_grid::index index_;
    };

    /// elevation index; quick and dirty; required for some applications
    class elevation_index
    {
        public:
            typedef double key_t;
            /// constructors
            elevation_index() {}
            elevation_index( double begin, double resolution ) : index_( 0, begin, resolution, resolution ) {}
            elevation_index( double resolution ) : index_( resolution, resolution ) {}

            /// @return index relative to begin with given resolution
            std::size_t operator()( double elevation ) const { return index_( 0, elevation )[1]; }

            /// @return begin
            double begin() const { return index_.begin().elevation(); }

            /// @return resolution
            double resolution() const { return index_.resolution().elevation(); }

            /// @return the lower and upper index between which value
            /// lies and corresponding weight from lower_index
            bounds get_bounds( const double value ) const;

        private:
            bearing_elevation_grid::index index_;
    };
};

/// spherical grid based on range-bearing-elevation coordinates
/// @todo implement based on voxel grid + bearing_elevation_grid
template < typename T >
class spherical_grid;

} // namespace snark {

#endif // SNARK_POINT_CLOUD_SPHERICAL_GRID_H_
