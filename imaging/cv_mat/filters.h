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
