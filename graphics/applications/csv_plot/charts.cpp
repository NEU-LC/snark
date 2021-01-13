// Copyright (c) 2021 Vsevolod Vlaskine

#include <QtCharts/QAbstractAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QtCore/QRandomGenerator>
#include <QtCore/QDebug>
#include "charts.h"

#include <iostream>

namespace snark { namespace graphics { namespace plotting {

chart::chart( float timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags ): QChart( QChart::ChartTypeCartesian, parent, window_flags )
{
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
    auto extents = std::make_pair( QPointF( std::numeric_limits< double >::max(), std::numeric_limits< double >::max() ), QPointF( std::numeric_limits< double >::min(), std::numeric_limits< double >::min() ) );
    for( unsigned int i = 0; i < streams_.size(); ++i )
    {
        streams_[i].update();
        if( !streams_[i].is_shutdown() ) { all_shutdown = false; }
        if( streams_[i].size() > 0 )
        {
            if( extents.first.x() > streams_[i].extents().first.x() ) { extents.first.setX( streams_[i].extents().first.x() ); }
            else if( extents.second.x() < streams_[i].extents().second.x() ) { extents.second.setX( streams_[i].extents().second.x() ); }
            if( extents.first.y() > streams_[i].extents().first.y() ) { extents.first.setY( streams_[i].extents().first.y() ); }
            else if( extents.second.y() < streams_[i].extents().second.y() ) { extents.second.setY( streams_[i].extents().second.y() ); }
        }
    }
    // todo: apply extents to axis ranges
    if( all_shutdown ) { timer_.stop(); }
}

xy_chart::xy_chart( float timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( timeout, parent, window_flags )
    , x_axis_( new QValueAxis )
    , y_axis_( new QValueAxis )
{
    addAxis( x_axis_, Qt::AlignBottom );
    addAxis( y_axis_, Qt::AlignLeft );
    x_axis_->setTickCount( 5 ); // todo!
    x_axis_->setRange( 0, 10 ); // todo!
    y_axis_->setRange( -5, 10 ); // todo!
}

void xy_chart::push_back( plotting::stream* s )
{
    streams_.push_back( s );
    addSeries( s->series );
    s->series->attachAxis( x_axis_ );
    s->series->attachAxis( y_axis_ );
}

} } } // namespace snark { namespace graphics { namespace plotting {
