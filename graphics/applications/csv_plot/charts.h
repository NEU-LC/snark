// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <QtCharts/QChart>
#include <QtCore/QTimer>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "stream.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class chart: public QChart
{
    Q_OBJECT
    public:
        chart( float timeout, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        virtual ~chart();
        virtual void push_back( plotting::stream* r ) = 0;
        void start();
        void shutdown();
        const boost::ptr_vector< plotting::stream >& streams() const { return streams_; }

    public slots:
        void update();
        
    protected:
        boost::ptr_vector< plotting::stream > streams_;
        boost::optional< std::pair< QPointF, QPointF > > extents_;
        virtual void update_() {}
        
    private:
        QTimer timer_;
};

class xy_chart: public chart
{
    Q_OBJECT
    public:
        xy_chart( float timeout, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        void push_back( plotting::stream* s );
        
    protected:
        void update_();
        
    private:
        QtCharts::QValueAxis* x_axis_;
        QtCharts::QValueAxis* y_axis_;
};

} } } // namespace snark { namespace graphics { namespace plotting {
