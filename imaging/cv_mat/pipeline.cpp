#include <snark/imaging/cv_mat/pipeline.h>
#include <tbb/tbb_thread.h>
#include <boost/bind.hpp>

namespace snark{ namespace imaging { namespace applications {

/// constructor
/// @param fields csv output fields
/// @param format csv output format
/// @param filters string describing the filters
/// @param size max buffer size
/// @param mode output mode
pipeline::pipeline( cv_mat::serialization& output, const std::string& filters, tbb::bursty_reader< pair >& reader ):
    m_output( output ),
    m_reader( reader )
{
    setup_pipeline_( filters );
}

/// write frame to std out
void pipeline::write_( pair p )
{
    if( p.second.size().width == 0 )
    {
        m_reader.stop();
        return;
    }
    if( std::cout.bad() || !std::cout.good() || is_shutdown_ )
    {
        m_reader.stop();
    }
    m_output.write( std::cout, p );
}

void pipeline::null_( pair p )
{
    if( p.second.size().width == 0 || std::cout.bad() || !std::cout.good() || is_shutdown_ )
    {
        m_reader.stop();
    }
}

/// setup the pipeline
/// @param filters name-value string describing the filters
void pipeline::setup_pipeline_( const std::string& filters )
{
    m_filters = snark::cv_mat::filters::make( filters );
    if( m_filters.empty() )
    {
        m_filter = ::tbb::filter_t< pair, void >( ::tbb::filter::serial_in_order, boost::bind( &pipeline::write_, this, _1 ) );
    }
    else
    {
        ::tbb::filter_t< pair, pair > allfilters;
        bool has_null = false;
        for( std::size_t i = 0; i < m_filters.size(); ++i )
        {
            ::tbb::filter::mode mode = ::tbb::filter::serial_in_order;
            if( m_filters[i].parallel )
            {
                mode = ::tbb::filter::parallel;
            }
            if( !m_filters[i].filter_function ) { has_null = true; break; }
            ::tbb::filter_t< pair, pair > filter( mode, boost::bind( m_filters[i].filter_function, _1 ) );
            allfilters = i == 0 ? filter : ( allfilters & filter );
        }
        m_filter = allfilters & ::tbb::filter_t< pair, void >( ::tbb::filter::serial_in_order, boost::bind( has_null ? &pipeline::null_ : &pipeline::write_, this, _1 ) );
    }
}

/// run the pipeline
void pipeline::run()
{
    m_pipeline.run( m_reader, m_filter );
}

} } }

