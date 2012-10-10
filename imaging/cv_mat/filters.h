#ifndef SNARK_IMAGING_CVMAT_FILTERS_H_
#define SNARK_IMAGING_CVMAT_FILTERS_H_

#include <vector>
#include <boost/function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/core/core.hpp>

namespace snark{ namespace cv_mat {

struct filter
{
    typedef std::pair< boost::posix_time::ptime, cv::Mat > value_type;
    filter( boost::function< value_type( value_type ) > f, bool p = true ): filter_function( f ), parallel( p ) {}
    boost::function< value_type( value_type ) > filter_function;
    bool parallel;
};

/// filter pipeline helpers
struct filters
{
    /// value type
    typedef std::pair< boost::posix_time::ptime, cv::Mat > value_type;

    /// return filters from name-value string
    static std::vector< filter > make( const std::string& how );

    /// apply filters (a helper)
    static value_type apply( std::vector< filter >& filters, value_type m );

    /// return filter usage
    static const std::string& usage();
};
    
} }  // namespace snark{ namespace cv_mat {

#endif // SNARK_IMAGING_CVMAT_FILTERS_H_
