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

#ifndef SNARK_IMAGING_APPLICATIONS_PIPELINE_H_
#define SNARK_IMAGING_APPLICATIONS_PIPELINE_H_

#ifdef WIN32
#include <winsock2.h>
#endif

#include <comma/application/signal_flag.h>
#include <snark/tbb/bursty_reader.h>
#include <snark/imaging/cv_mat/bursty_pipeline.h>
#include <snark/imaging/cv_mat/serialization.h>
#include <snark/imaging/cv_mat/filters.h>

namespace snark {
    
namespace tbb {

template<>
struct bursty_reader_traits< std::pair< boost::posix_time::ptime, cv::Mat > >
{
    static bool valid( const std::pair< boost::posix_time::ptime, cv::Mat >& p ) { return ( p.second.size().width > 0 ); }
};

}

namespace imaging { namespace applications {

/// base class for video processing, capture images in a serarate thread, apply filters, serialize to stdout
class pipeline
{
public:
    typedef std::pair< boost::posix_time::ptime, cv::Mat > pair;
    pipeline( cv_mat::serialization& output, const std::string& filters, tbb::bursty_reader< pair >& reader );

    void run();
    
protected:
    void write_( pair p );
    void null_( pair p );
    void setup_pipeline_( const std::string& filters );

    cv_mat::serialization& m_output;
    ::tbb::filter_t< pair, void > m_filter;
    std::vector< cv_mat::filter > m_filters;
    tbb::bursty_reader< pair >& m_reader;
    tbb::bursty_pipeline< pair > m_pipeline;
    comma::signal_flag is_shutdown_;
};


} } } 

#endif // SNARK_IMAGING_APPLICATIONS_PIPELINE_H_
