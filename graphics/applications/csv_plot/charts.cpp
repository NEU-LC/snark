// Copyright (c) 2021 Vsevolod Vlaskine

#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "charts.h"

namespace snark { namespace graphics { namespace plotting {

chart::chart( float timeout, const std::string& title, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : QChart( QChart::ChartTypeCartesian, parent, window_flags )
    , title_( title )
{
    setTitle( &title[0] );
    legend()->hide();
    setAnimationOptions( QChart::SeriesAnimations ); // chart->setAnimationOptions( QChart::AllAnimations ); // todo? make configurable?
    QObject::connect( &timer_, &QTimer::timeout, this, &chart::update );
    timer_.setInterval( ( unsigned int )( timeout * 1000 ) );
}

chart::~chart() { shutdown(); }

void chart::start()
{
    for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].start(); }
    timer_.start();
}

void chart::shutdown()
{
    timer_.stop();
    for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].shutdown(); }
}

void chart::update()
{
    bool all_shutdown = true;
    for( unsigned int i = 0; i < streams_.size(); ++i )
    {
        streams_[i].update();
        if( !streams_[i].is_shutdown() ) { all_shutdown = false; }
    }
    update_();
    if( all_shutdown ) { timer_.stop(); }
}

xy_chart::xy_chart( float timeout, const std::string& title, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( timeout, title, parent, window_flags )
    , x_axis_( new QValueAxis )
    , y_axis_( new QValueAxis )
    , scroll_( false )
{
    addAxis( x_axis_, Qt::AlignBottom );
    addAxis( y_axis_, Qt::AlignLeft );
    x_axis_->setTickCount( 1 ); // todo!
    x_axis_->setRange( 0, 10 ); // todo!
    y_axis_->setRange( 0, 10 ); // todo!
}

void xy_chart::push_back( plotting::stream* s )
{
    streams_.push_back( s );
    addSeries( s->series );
    s->series->attachAxis( x_axis_ );
    s->series->attachAxis( y_axis_ );
    if( s->config.series.scroll ) { scroll_ = true; } // quick and dirty
}

void xy_chart::update_()
{
    // todo: handle range of zero length
    // todo: add configurable margins
    // todo: handle various range policies
    // todo: fixed range
    extents_.reset();
    for( unsigned int i = 0; i < streams_.size(); ++i )
    {
        if( streams_[i].size() == 0 ) { continue; }
        if( !extents_ ) { extents_ = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() ), QPointF( std::numeric_limits< double >::min(), std::numeric_limits< double >::min() ) ); }
        if( extents_->first.x() > streams_[i].extents().first.x() ) { extents_->first.setX( streams_[i].extents().first.x() ); }
        if( extents_->second.x() < streams_[i].extents().second.x() ) { extents_->second.setX( streams_[i].extents().second.x() ); }
        if( extents_->first.y() > streams_[i].extents().first.y() ) { extents_->first.setY( streams_[i].extents().first.y() ); }
        if( extents_->second.y() < streams_[i].extents().second.y() ) { extents_->second.setY( streams_[i].extents().second.y() ); }
    }
    if( !extents_ ) { return; }
    if( scroll_ ) // todo! quick and dirty; improve
    {
        double mx = ( extents_->second.x() - extents_->first.x() ) * 0.1;
        double my = ( extents_->second.y() - extents_->first.y() ) * 0.1;
        x_axis_->setMin( extents_->first.x() - mx );
        x_axis_->setMax( extents_->second.x() + mx );
        y_axis_->setMin( extents_->first.y() - my );
        y_axis_->setMax( extents_->second.y() + my );
    }
    else
    {
        double mx = ( x_axis_->max() - x_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
        double my = ( y_axis_->max() - y_axis_->min() ) * 0.1; // todo: make configurable or at least not so daft
        if( x_axis_->min() > extents_->first.x() ) { x_axis_->setMin( extents_->first.x() - mx ); }
        if( x_axis_->max() < extents_->second.x() ) { x_axis_->setMax( extents_->second.x() + mx ); }
        if( y_axis_->min() > extents_->first.y() ) { y_axis_->setMin( extents_->first.y() - my ); }
        if( y_axis_->max() < extents_->second.y() ) { y_axis_->setMax( extents_->second.y() + my ); }
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {
