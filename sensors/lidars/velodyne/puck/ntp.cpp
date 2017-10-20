#include "ntp.h"

namespace snark {  namespace velodyne { namespace puck {
    
    boost::posix_time::ptime ntp_t::update_timestamp(const boost::posix_time::ptime& t, comma::uint32 ntp)
    {
        const boost::posix_time::ptime base( comma::csv::impl::epoch );
        const boost::posix_time::time_duration d = t - base;
        // get system time: number of microseconds since start of the hour
        comma::uint64 ms = d.total_microseconds() % ( 3600000000 );
        // if system time has rolled over to 0 and ntp has not, add an hour
        // ( system time is ahead of ntp )
        if ( ms < ntp ) { ms += 3600000000; }
        comma::int64 diff = ms - ntp;
        if ( diff > threshold_ * 1000000.0 )
        {
            // the values are more than one second apart - time in packet wasn't set correctly.
            if (!permissive_) 
            { 
                COMMA_THROW(comma::exception, "cannot match system time with ntp time, difference " << diff << " is greater than threshold " << threshold_ ); 
            }
            return boost::posix_time::not_a_date_time;
        }
        return t - boost::posix_time::microseconds(ms) + boost::posix_time::microseconds(ntp);
    }

} } } // namespace snark {  namespace velodyne { namespace puck {
