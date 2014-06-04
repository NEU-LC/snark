#ifndef SNARK_BATTERY_OCEAN_IO_H
#define SNARK_BATTERY_OCEAN_IO_H

namespace snark { namespace ocean {

template < typename Derived >
struct query_io
{
    void set_timeout( const boost::posix_time::time_duration& duration )  {  
        return static_cast< Derived* >( this )->set_timeout_impl( duration );
    }

    ocean8 read() {
        return static_cast< Derived* >( this )->read_impl();
    }
    
    void write( ocean8 value ) {
        return static_cast< Derived* >( this )->write_impl( value );
    };

};


} } // namespace snark { namespace ocean {


#endif // SNARK_BATTERY_OCEAN_IO_H