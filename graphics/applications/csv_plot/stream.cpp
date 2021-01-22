// Copyright (c) 2021 Vsevolod Vlaskine

#include <limits>
#include <type_traits>
#include <boost/bind.hpp>
#include <QtCharts/QChart>
#include <comma/base/exception.h>
#include "stream.h"
#include "traits.h"

namespace snark { namespace graphics { namespace plotting {

plotting::stream* stream::make( const plotting::stream::config_t& config, const std::map< std::string, snark::graphics::plotting::chart* >& charts )
{
    auto master_series = plotting::series::xy::make( charts.find( config.series.chart )->second, config.series );
    plotting::record sample = plotting::record::sample( config.csv.fields, config.number_of_series );
    std::vector< plotting::series::xy > series;
    for( unsigned int i = 0; i < sample.series.size(); ++i ) { series.push_back( plotting::series::xy::make( charts.find( config.series.chart )->second, config.series ) ); } // todo: series configs
    return new plotting::stream( master_series, series, config );
}
    
stream::config_t::config_t( const comma::command_line_options& options )
    : csv( options, "x,y" )
    , pass_through( options.exists( "--pass-through,--pass" ) )
    , series( options )
    , size( options.value( "--size,-s,--tail", 10000 ) )
    , number_of_series( options.value( "--number-of-series,-n", 0 ) )
{
}

stream::stream( const plotting::series::xy& m, const std::vector< plotting::series::xy >& s, const config_t& config )
    : master_series( m )
    , series( s )
    , config( config )
    , is_shutdown_( false )
    , is_stdin_( config.csv.filename == "-" )
    , is_( config.csv.filename, config.csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking )
    , istream_( *is_, config.csv, plotting::record::sample( config.csv.fields, config.number_of_series ) )
    , count_( 0 )
    , has_x_( config.csv.fields.empty() || config.csv.has_field( "x" ) )
    , buffers_( config.size )
    , size_( 0 )
{
    if( config.pass_through ) { passed_.reset( new comma::csv::passed< graphics::plotting::record >( istream_, std::cout, config.csv.flush ) ); }
}

stream::buffers_t_::buffers_t_( comma::uint32 size ) : records( size ) {}

void stream::buffers_t_::add( const record& p ) { records.add( p, p.block ); }

bool stream::buffers_t_::changed() const { return records.changed(); }

void stream::buffers_t_::mark_seen() { records.mark_seen(); }

void stream::start() { thread_.reset( new boost::thread( boost::bind( &graphics::plotting::stream::read_, boost::ref( *this ) ) ) ); }

bool stream::is_shutdown() const { return is_shutdown_; }

bool stream::is_stdin() const { return is_stdin_; }

void stream::shutdown()
{
    is_shutdown_ = true;
    if( thread_ ) { thread_->join(); }
    if( !is_stdin_ ) { is_.close(); }
}

void stream::read_()
{
    while( !is_shutdown_ && ( istream_.ready() || ( is_->good() && !is_->eof() ) ) )
    {
        const record* p = istream_.read();
        if( !p ) { break; }
        if( passed_ ) { passed_->write(); }
        record q = *p;
        if( !has_x_ ) { q.x = count_; }
        ++count_;
        comma::synchronized< records_t >::scoped_transaction( records )->push_back( q );
    }
    is_shutdown_ = true;
}

bool stream::update()
{
    records_t p;
    {
        comma::synchronized< records_t >::scoped_transaction t( records );
        p = *t; // todo! quick and dirty, potentially massive copy; watch performance!
        t->clear();
    }
    if( p.empty() ) { return false; }
    for( auto& e: p ) { buffers_.add( e ); }
    bool changed = buffers_.changed();
    static_assert( sizeof( qreal ) == 8 );
    size_ = buffers_.records.size();
    auto append = [&]( plotting::series::xy& s, unsigned int i ) { s.append( buffers_.records.values()[i].t, buffers_.records.values()[i] ); }; // todo: support 3d data, time series, polar data
    auto update_series = [&]( plotting::series::xy& s )
    {
        s.clear();
        for( unsigned int i = buffers_.records.begin(); i < buffers_.records.size(); ++i ) { append( s, i ); }
        for( unsigned int i = 0; i < buffers_.records.begin(); ++i ) { append( s, i ); }
    };
    if( buffers_.changed() )
    {
        update_series( master_series );
        for( auto& s: series ) { update_series( s ); }
    }
    buffers_.mark_seen();
    return changed;
}

} } } // namespace snark { namespace graphics { namespace plotting {
