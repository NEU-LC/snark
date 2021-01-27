// Copyright (c) 2021 Vsevolod Vlaskine

#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "charts.h"

namespace snark { namespace graphics { namespace plotting {

chart::config_t::config_t( const std::string& name, const std::string& t )
    : animate( true )
    , legend( false )
    , name( name )
    , scroll( false )
    , title( t.empty() ? name : t )
{
}

chart::config_t chart::config_t::make( const std::string& s )
{
    COMMA_THROW( comma::exception, "todo" );
}

std::vector< chart::config_t > chart::config_t::make( const std::vector< std::string >& v )
{
    COMMA_THROW( comma::exception, "todo" );
}
    
chart::chart( const chart::config_t& config, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : QChart( QChart::ChartTypeCartesian, parent, window_flags )
    , config_( config )
    , fixed_x_( config.min.x && config.max.x )
    , fixed_y_( config.min.y && config.max.y )
{
    setTitle( &config_.title[0] );
    if( !config_.legend ) { legend()->hide(); }
    if( config_.animate ) { setAnimationOptions( QChart::SeriesAnimations ); }
}

xy_chart::xy_chart( const chart::config_t& config, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( config, parent, window_flags )
    , x_axis_( new QValueAxis )
    , y_axis_( new QValueAxis )
{
    addAxis( x_axis_, Qt::AlignBottom ); // todo: make configurable
    addAxis( y_axis_, Qt::AlignLeft ); // todo: make configurable
    x_axis_->setTickCount( 1 ); // todo: make configurable
    x_axis_->setRange( config.min.x ? *config.min.x : 0, config.min.y ? *config.min.y : 10 ); // todo: configure
    y_axis_->setRange( config.max.x ? *config.max.x : 0, config.max.y ? *config.max.y : 10 ); // todo: configure
}

void xy_chart::push_back( plotting::series::xy* s )
{
    series_.push_back( s );
    addSeries( ( *s )() );
    ( *s )()->attachAxis( x_axis_ );
    ( *s )()->attachAxis( y_axis_ );
    if( s->config().scroll ) { config_.scroll = true; } // quick and dirty
}

void xy_chart::update()
{
    if( fixed_x_ && fixed_y_ ) { return; }
    extents_.reset();
    for( auto s: series_ )
    {
        if( !s->updated() ) { continue; }
        if( !extents_ ) { extents_ = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() ), QPointF( std::numeric_limits< double >::min(), std::numeric_limits< double >::min() ) ); }
        if( !fixed_x_ )
        {
            if( extents_->first.x() > s->extents().first.x() ) { extents_->first.setX( s->extents().first.x() ); }
            if( extents_->second.x() < s->extents().second.x() ) { extents_->second.setX( s->extents().second.x() ); }
        }
        if( !fixed_y_ )
        {
            if( extents_->first.y() > s->extents().first.y() ) { extents_->first.setY( s->extents().first.y() ); }
            if( extents_->second.y() < s->extents().second.y() ) { extents_->second.setY( s->extents().second.y() ); }
        }
    }
    if( !extents_ ) { return; }
    if( !fixed_x_ )
    {
        if( config_.scroll ) // todo! quick and dirty; improve
        {
            double mx = ( extents_->second.x() - extents_->first.x() ) * 0.1;
            x_axis_->setMin( extents_->first.x() - mx );
            x_axis_->setMax( extents_->second.x() + mx );
        }
        else
        {
            double mx = ( x_axis_->max() - x_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
            if( x_axis_->min() > extents_->first.x() ) { x_axis_->setMin( extents_->first.x() - mx ); }
            if( x_axis_->max() < extents_->second.x() ) { x_axis_->setMax( extents_->second.x() + mx ); }
        }
    }
    if( !fixed_y_ )
    {
        if( config_.scroll ) // todo! quick and dirty; improve
        {
            double my = ( extents_->second.y() - extents_->first.y() ) * 0.1;
            y_axis_->setMin( extents_->first.y() - my );
            y_axis_->setMax( extents_->second.y() + my );
        }
        else
        {
            double my = ( y_axis_->max() - y_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
            if( y_axis_->min() > extents_->first.y() ) { y_axis_->setMin( extents_->first.y() - my ); }
            if( y_axis_->max() < extents_->second.y() ) { y_axis_->setMax( extents_->second.y() + my ); }
        }
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {
