// Copyright (c) 2021 Vsevolod Vlaskine

#include <boost/bind.hpp>
#include <QtCharts/QXYSeries>
#include "series.h"
#include "traits.h"

namespace snark { namespace graphics { namespace plotting {

static const char* hex_color_( const std::string& c )
{
    if( c == "red" ) { return "#FF0000"; }
    if( c == "green" ) { return "#00FF00"; }
    if( c == "blue" ) { return "#0000FF"; }
    if( c == "yellow" ) { return "#FFFF00"; }
    if( c == "cyan" ) { return "#00FFFF"; }
    if( c == "magenta" ) { return "#FF00FF"; }
    if( c == "black" ) { return "#000000"; }
    if( c == "white" ) { return "#FFFFFF"; }
    if( c == "grey" ) { return "#888888"; }
    return &c[0];
}
    
series::config_t::config_t( const comma::command_line_options& options )
    : csv( options )
    , size( options.value( "--size,-s,--tail", 10000 ) )
    , color_name( options.value< std::string >( "--color,--colour", "black" ) )
    , shape( options.value< std::string >( "--shape,--type", "line" ) )
    , style( options.value< std::string >( "--style", "" ) )
    , weight( options.value( "--weight", 0.0 ) )
{
    if( csv.fields.empty() ) { csv.fields="x,y"; } // todo: parametrize on the graph type
    if( !color_name.empty() ) { color = QColor( hex_color_( color_name ) ); }
}

series::series( QXYSeries* s, const config_t& config )
    : series_todo( s )
    , config( config )
    , is_shutdown_( false )
    , is_stdin_( config.csv.filename == "-" )
    , is_( config.csv.filename, config.csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking )
    , istream_( *is_, config.csv )
    , count_( 0 )
    , has_x_( config.csv.fields.empty() || config.csv.has_field( "x" ) )
    , buffers_( config.size )
{
    QPen pen( config.color );
    pen.setWidth( config.weight );
    series_todo->setPen( pen );
}

series::buffers_t_::buffers_t_( comma::uint32 size ) : points( size ) {}

void series::buffers_t_::add( const point& p ) { points.add( QPointF( p.coordinates.x(), p.coordinates.y() ), p.block ); }

bool series::buffers_t_::changed() const { return points.changed(); }

void series::buffers_t_::mark_seen() { points.mark_seen(); }

void series::start() { thread_.reset( new boost::thread( boost::bind( &graphics::plotting::series::read_, boost::ref( *this ) ) ) ); }

bool series::is_shutdown() const { return is_shutdown_; }

bool series::is_stdin() const { return is_stdin_; }

void series::shutdown()
{
    is_shutdown_ = true;
    if( thread_ ) { thread_->join(); }
    if( !is_stdin_ ) { is_.close(); }
}

void series::read_()
{
    while( !is_shutdown_ && ( istream_.ready() || ( is_->good() && !is_->eof() ) ) )
    {
        const point* p = istream_.read();
        if( !p ) { break; }
        point q = *p;
        if( !has_x_ ) { q.coordinates.x() = count_; }
        ++count_;
        comma::synchronized< points_t >::scoped_transaction( points )->push_back( q );
    }
    is_shutdown_ = true;
}

bool series::update()
{
    points_t p;
    {
        comma::synchronized< points_t >::scoped_transaction t( points );
        p = *t; // todo! quick and dirty, potentially massive copy; watch performance!
        t->clear();
    }
    if( p.empty() ) { return false; }
    for( auto& e: p ) { buffers_.add( e ); }
    bool changed = buffers_.changed();
    if( buffers_.changed() )
    {
        series_todo->clear();
        for( auto& e: buffers_.points.values() ) { series_todo->append( e ); } // todo! super-quick and dirty; massively inefficient
    }
    buffers_.mark_seen();
    return changed;
}

} } } // namespace snark { namespace graphics { namespace plotting {
