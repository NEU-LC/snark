// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <vector>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "record.h"
#include "series.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class chart: public QChart
{
    Q_OBJECT
    public:
        struct config_t
        {
            bool animate;
            bool legend;
            point max; // todo? is it xychart-specific?
            point min; // todo? is it xychart-specific?
            std::string name;
            bool scroll; // todo? a better name?
            std::string title;
            
            config_t( const std::string& name = "", const std::string& title = "" );
            static config_t make( const std::string& s );
            static std::vector< config_t > make( const std::vector< std::string >& v );
        };
        
        chart( const config_t& c, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        virtual ~chart() {}
        virtual void push_back( plotting::series::xy* r ) = 0;
        virtual void update() = 0;
        const std::vector< plotting::series::xy* >& series() const { return series_; }
        const config_t& config() const { return config_; }
        
        
    protected:
        std::vector< plotting::series::xy* > series_; // todo! implement and use series::base!
        config_t config_;
        bool fixed_x_; // quick and dirty
        bool fixed_y_; // quick and dirty
        virtual void update_() {}
        
    private:
        std::string title_;
};

class xy_chart: public chart
{
    Q_OBJECT
    public:
        xy_chart( const chart::config_t& config, QGraphicsItem *parent = nullptr, Qt::WindowFlags window_flags = {} );
        void push_back( plotting::series::xy* s );        
        void update();
        
    private:
        boost::optional< std::pair< QPointF, QPointF > > extents_;
        QtCharts::QValueAxis* x_axis_;
        QtCharts::QValueAxis* y_axis_;
};

} } } // namespace snark { namespace graphics { namespace plotting {
