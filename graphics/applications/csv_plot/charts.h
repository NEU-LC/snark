// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "series.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class chart: public QChart
{
    Q_OBJECT
    public:
        chart( const std::string& title, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        virtual ~chart() {}
        virtual void push_back( plotting::series::xy* r ) = 0;
        virtual void update() = 0;
        const std::vector< plotting::series::xy* >& series() const { return series_; }
        const std::string& title() const { return title_; }
        
        
    protected:
        std::vector< plotting::series::xy* > series_; // todo! implement and use series::base!
        virtual void update_() {}
        
    private:
        std::string title_;
};

class xy_chart: public chart
{
    Q_OBJECT
    public:
        xy_chart( const std::string& title, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        void push_back( plotting::series::xy* s );        
        void update();
        
    private:
        boost::optional< std::pair< QPointF, QPointF > > extents_;
        QtCharts::QValueAxis* x_axis_;
        QtCharts::QValueAxis* y_axis_;
        bool scroll_; // todo! a better name!
};

} } } // namespace snark { namespace graphics { namespace plotting {
