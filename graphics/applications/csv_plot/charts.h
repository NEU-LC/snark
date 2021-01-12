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
        chart( double timeout, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        virtual ~chart() {}
        void push_back( plotting::series* r );
        void start();
        void shutdown();

    public slots:
        void update();
        
    protected:
        virtual void update_() = 0;

    private:
        QTimer timer_;
        QtCharts::QValueAxis *x_axis_;
        QtCharts::QValueAxis *y_axis_;
        QtCharts::QLineSeries *qtseries_; // todo
        boost::ptr_vector< plotting::series > series_;
        double timeout_;
};

class line_chart: public chart
{
    Q_OBJECT
    public:
        line_chart( double timeout, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );

    protected:
        void update_();
};

} } } // namespace snark { namespace graphics { namespace plotting {
