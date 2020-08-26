// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/types.h>

namespace snark { namespace cv_mat { namespace filters { namespace partitions {

template < typename H >
class partition
{
    public:
        explicit partition(boost::optional< cv::Scalar > do_not_visit_value = boost::optional< cv::Scalar >(),
                           comma::int32 none = -1, bool merge = false, bool keep_id = false,
                           comma::int32 start_from = 0, unsigned int min_partition_size = 0, unsigned int degrees = 8);

        std::pair< H, cv::Mat > operator()(std::pair< H, cv::Mat > m);

        typedef boost::function< std::pair< H, cv::Mat >(std::pair< H, cv::Mat >) > functor_t;

        /// return partition functor and boolean flag that indicates whether functor is safely re-entrant in multithread context
        /// functor is re-entrant, if keep_id is set to false
        /// @todo? protect with mutex instead?
        static std::pair< functor_t, bool > make(const std::string &options);

        static std::string usage(unsigned int indent = 0);

    private:
        boost::optional< cv::Scalar > do_not_visit_value_;
        comma::int32 none_;
        bool merge_;
        bool keep_id_;
        comma::int32 start_from_;
        comma::int32 id_;
        unsigned int min_partition_size_;
        unsigned int degrees_;
};

template < typename H >
class reduce
{
    public:
        explicit reduce( unsigned int channel = 0
                       , comma::int32 background = -1
                       , bool merge = false
                       , unsigned int colours = 6 ) : channel_( channel ), background_( background ), merge_( merge ), colours_( colours ) {};

        std::pair< H, cv::Mat > operator()(std::pair< H, cv::Mat > m);

        typedef boost::function< std::pair< H, cv::Mat >(std::pair< H, cv::Mat >) > functor_t;

        static std::pair< functor_t, bool > make( const std::string &options );

        static std::string usage(unsigned int indent = 0);

    private:
        unsigned int channel_;
        comma::int32 background_;
        bool merge_;
        unsigned int colours_;

        template < typename T, int I > cv::Mat process_( cv::Mat m, int type );
};

} } } } // namespace snark { namespace cv_mat { namespace impl { namespace partitions {
