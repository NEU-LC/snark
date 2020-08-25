// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2019 Vsevolod Vlaskine

#pragma once

#include <string>
#include <vector>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include "../serialization.h"

namespace snark { namespace cv_mat { namespace filters {
    
template < typename H >
class file
{
    public:
        typedef boost::function< boost::posix_time::ptime( const H& ) > get_timestamp_functor;

        file( const get_timestamp_functor& get_timestamp
            , const std::string& type
            , bool no_header = false
            , const boost::optional< int >& quality = boost::none
            , bool do_index = false
            , bool numbered = false
            , bool force_filenames = false
            , bool exit_if_done = false
            , const std::vector< std::string >& filenames = std::vector< std::string >()
            , const std::vector< std::pair< unsigned int, unsigned int > >& ranges = std::vector< std::pair< unsigned int, unsigned int > >() );

        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        
        typedef boost::function< std::pair< H, cv::Mat >( std::pair< H, cv::Mat > ) > functor_t;
        
        static std::pair< functor_t, bool > make( boost::function< boost::posix_time::ptime( const H& ) > get_timestamp, const std::string& options );
    
        static std::string usage( unsigned int indent = 0 );
        
    private:
        const get_timestamp_functor get_timestamp_;
        std::string type_;
        boost::optional< int > quality_;
        bool do_index_;
        bool numbered_;
        bool force_filenames_;
        bool exit_if_done_;
        snark::cv_mat::serialization serialization_;
        boost::posix_time::ptime previous_timestamp_;
        unsigned int index_;
        std::vector< std::string > filenames_;
        unsigned int filename_index_;
        std::vector< std::pair< unsigned int, unsigned int > > ranges_;
        unsigned int range_index_;
        unsigned int count_;
        std::string make_filename_( const boost::posix_time::ptime& t );
};

} } } // namespace snark { namespace cv_mat { namespace impl {
