// Copyright (c) 2021 Vsevolod Vlaskine

#include <QtCharts/QAbstractAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QtCore/QRandomGenerator>
#include <QtCore/QDebug>
#include "charts.h"

namespace snark { namespace graphics { namespace plotting {

chart::chart( float timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags ): QChart( QChart::ChartTypeCartesian, parent, window_flags )
{
    QObject::connect( &timer_, &QTimer::timeout, this, &chart::update );
    timer_.setInterval( ( unsigned int )( timeout * 1000 ) );
}

chart::~chart() { shutdown(); }

void chart::start() { timer_.start(); }

void chart::shutdown()
{
    timer_.stop();
    for( unsigned int i = 0; i < series_.size(); ++i ) { series_[i].shutdown(); }
}

void chart::update()
{
    bool all_shutdown = true;
    for( unsigned int i = 0; i < series_.size(); ++i )
    {
        series_[i].update();
        if( !series_[i].is_shutdown() ) { all_shutdown = false; }
    }
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

void xy_chart::push_back( plotting::series* s )
{
    series_.push_back( s );
    addSeries( s->series_todo );
    s->series_todo->attachAxis( x_axis_ );
    s->series_todo->attachAxis( y_axis_ );
}

} } } // namespace snark { namespace graphics { namespace plotting {
