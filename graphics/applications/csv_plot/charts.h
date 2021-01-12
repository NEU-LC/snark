// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <QtCharts/QChart>
#include <QtCore/QTimer>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <boost/ptr_container/ptr_vector.hpp>
#include "series.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class chart: public QChart
{
    Q_OBJECT
    public:
        chart( float timeout, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        virtual ~chart();
        virtual void push_back( plotting::series* r ) = 0;
        void start();
        void shutdown();
        const boost::ptr_vector< plotting::series >& series() const { return series_; }

    public slots:
        void update();
        
    protected:
        boost::ptr_vector< plotting::series > series_;
        
    private:
        QTimer timer_;
};

class xy_chart: public chart
{
    Q_OBJECT
    public:
        xy_chart( float timeout, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        void push_back( plotting::series* s );
        
    private:
        QtCharts::QValueAxis* x_axis_;
        QtCharts::QValueAxis* y_axis_;
};

} } } // namespace snark { namespace graphics { namespace plotting {
