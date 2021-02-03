// Copyright (c) 2021 Vsevolod Vlaskine

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

chart::config_t::config_t( const comma::command_line_options& options ) : config_t()
{
    scroll = options.exists( "--scroll" ); // todo? more options?
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
    //setMargins( QMargins( 5, 5, 5, 5 ) ); // quick and dirty
    //QFont font( x_axis_->titleFont().family(), 1, 1 );
    x_axis_->setTitleFont( x_axis_->titleFont() ); // voodoo, this removes font boldness... whatever...
    y_axis_->setTitleFont( y_axis_->titleFont() ); // voodoo, this removes font boldness... whatever...
    x_axis_->setTitleText( &config.axes.x.title[0] );
    y_axis_->setTitleText( &config.axes.y.title[0] );
    addAxis( x_axis_, Qt::AlignBottom ); // todo? make configurable
    addAxis( y_axis_, Qt::AlignLeft ); // todo? make configurable
    x_axis_->setTickCount( 1 ); // todo: make configurable
    x_axis_->setRange( config.min.x ? *config.min.x : 0, config.max.x ? *config.max.x : 10 );
    y_axis_->setRange( config.min.y ? *config.min.y : 0, config.max.y ? *config.max.y : 10 );
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
        if( !config_.min.x && extents_->first.x() > s->extents().first.x() ) { extents_->first.setX( s->extents().first.x() ); }
        if( !config_.max.x && extents_->second.x() < s->extents().second.x() ) { extents_->second.setX( s->extents().second.x() ); }
        if( !config_.min.y && extents_->first.y() > s->extents().first.y() ) { extents_->first.setY( s->extents().first.y() ); }
        if( !config_.max.y && extents_->second.y() < s->extents().second.y() ) { extents_->second.setY( s->extents().second.y() ); }
    }
    if( !extents_ ) { return; }
    if( !fixed_x_ )
    {
        if( config_.scroll ) // todo! quick and dirty; improve
        {
            double mx = ( extents_->second.x() - extents_->first.x() ) * 0.1;
            if( !config_.min.x ) { x_axis_->setMin( extents_->first.x() - mx ); }
            if( !config_.max.x ) { x_axis_->setMax( extents_->second.x() + mx ); }
        }
        else
        {
            double mx = ( x_axis_->max() - x_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
            if( !config_.min.x && x_axis_->min() > extents_->first.x() ) { x_axis_->setMin( extents_->first.x() - mx ); }
            if( !config_.max.x && x_axis_->max() < extents_->second.x() ) { x_axis_->setMax( extents_->second.x() + mx ); }
        }
    }
    if( !fixed_y_ )
    {
        if( config_.scroll ) // todo! quick and dirty; improve
        {
            double my = ( extents_->second.y() - extents_->first.y() ) * 0.1;
            if( !config_.min.y ) { y_axis_->setMin( extents_->first.y() - my ); }
            if( !config_.max.y ) { y_axis_->setMax( extents_->second.y() + my ); }
        }
        else
        {
            double my = ( y_axis_->max() - y_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
            if( !config_.min.y && y_axis_->min() > extents_->first.y() ) { y_axis_->setMin( extents_->first.y() - my ); }
            if( !config_.max.y && y_axis_->max() < extents_->second.y() ) { y_axis_->setMax( extents_->second.y() + my ); }
        }
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {
