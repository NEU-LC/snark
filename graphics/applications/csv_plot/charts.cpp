// Copyright (c) 2021 Vsevolod Vlaskine

#include <QtCharts/QAbstractAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QtCore/QRandomGenerator>
#include <QtCore/QDebug>
#include "charts.h"

namespace snark { namespace graphics { namespace plotting {

chart::chart( double timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : QChart( QChart::ChartTypeCartesian, parent, window_flags )
    , x_axis_( new QValueAxis() )
    , y_axis_( new QValueAxis() )
    , series_( 0 )
    , timeout_( timeout )
{
    series_ = new QLineSeries( this ); // series_ = new QSplineSeries( this ); // todo
    QPen pen( Qt::red ); // todo
    pen.setWidth( 3 ); // todo
    series_->setPen( pen );
    series_->append( 0, 0 ); // todo
    addSeries( series_ );
    addAxis( x_axis_, Qt::AlignBottom );
    addAxis( y_axis_, Qt::AlignLeft );
    series_->attachAxis( x_axis_ );
    series_->attachAxis( y_axis_ );
    x_axis_->setTickCount( 5 ); // todo
    x_axis_->setRange( 0, 10 ); // todo
    y_axis_->setRange( -5, 10 ); // todo
    QObject::connect( &timer_, &QTimer::timeout, this, &chart::update );
}

void chart::start()
{
    timer_.setInterval( ( unsigned int )( timeout_ * 1000 ) );
    timer_.start();
}

void chart::shutdown() { for( unsigned int i = 0; i < streams_.size(); ++i ) { streams_[i].shutdown(); } }

chart::~chart() {}

void chart::update()
{
    // todo: if all streams finished, stop timer
    // todo: update timeseries from the streams
    update_();
}

line_chart::line_chart( double timeout, QGraphicsItem *parent, Qt::WindowFlags window_flags )
    : chart( timeout, parent, window_flags )
{
}

void line_chart::update_()
{
    // todo
}

// void chart::update()
// {
//     std::cerr << "--> tick" << std::endl;
//     qreal x = plotArea().width() / m_axisX->tickCount();
//     qreal y = (m_axisX->max() - m_axisX->min()) / m_axisX->tickCount();
//     m_x += y;
//     m_y = QRandomGenerator::global()->bounded(5) - 2.5;
//     series_->append(m_x, m_y);
//     scroll(x, 0);
//     if (m_x == 100)
//         .stop();
// }

} } } // namespace snark { namespace graphics { namespace plotting {
