// Copyright (c) 2021 Vsevolod Vlaskine

#include <QtCharts/QXYSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QSplineSeries>
#include <comma/base/exception.h>
#include "series.h"

namespace snark { namespace graphics { namespace plotting { namespace series {

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
    
config::config( const comma::command_line_options& options )
    : color_name( options.value< std::string >( "--color,--colour", "black" ) )
    , color( color_name.empty() ? QColor( 0, 0, 0 ) : QColor( hex_color_( color_name ) ) )
    , scroll( options.exists( "--scroll" ) )
    , shape( options.value< std::string >( "--shape,--type", "line" ) )
    , style( options.value< std::string >( "--style", "" ) )
    , title( options.value< std::string >( "--title", "" ) )
    , weight( options.value( "--weight", 1.0 ) )
{
}

xy::xy( QtCharts::QXYSeries* s, const series::config& c ): series_( s ), config_( c ), updated_( false )
{
    QPen pen( config_.color );
    pen.setWidth( config_.weight );
    series_->setPen( pen );
    series_->setName( &config_.title[0] );
    series_->setColor( config_.color );
}

void xy::clear()
{
    series_->clear();
    extents_ = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() )
                             , QPointF( std::numeric_limits< double >::lowest(), std::numeric_limits< double >::lowest() ) );
    updated_ = false;
}

void xy::append( boost::posix_time::ptime, const point& p )
{
    series_->append( QPointF( *p.x, *p.y ) ); // todo: support 3d data, time series, polar data (or template stream upon those)
    if( extents_.first.x() > p.x ) { extents_.first.setX( *p.x ); }
    if( extents_.second.x() < p.x ) { extents_.second.setX( *p.x ); }
    if( extents_.first.y() > p.y ) { extents_.first.setY( *p.y ); }
    if( extents_.second.y() < p.y ) { extents_.second.setY( *p.y ); }
    updated_ = true;
}

QtCharts::QXYSeries* make_series_( const series::config& c, QtCharts::QChart* chart )
{
    if( c.shape == "line" || c.shape.empty() ) { return new QtCharts::QLineSeries( chart ); }
    if( c.shape == "spline" ) { return new QtCharts::QSplineSeries( chart ); }
    if( c.shape == "scatter" ) // todo: quick and dirty; make polymorphic
    {
        auto s = new QtCharts::QScatterSeries( chart );
        s->setMarkerSize( c.weight );
        s->setBorderColor( c.color );
        return s;
    }
    COMMA_THROW( comma::exception, "csv-plot: expected stream type as shape, got: \"" << c.shape << "\"" );
};

bool xy::updated( bool reset )
{ 
    bool r = updated_;
    updated_ = updated_ && !reset;
    return r;
}

xy xy::make( const series::config& c, QtCharts::QChart* chart ) { return xy( make_series_( c, chart ), c ); }

} } } } // namespace snark { namespace graphics { namespace plotting { namespace series {
