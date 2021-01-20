// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <QColor>
#include <QtCharts/QXYSeries>
#include <comma/application/command_line_options.h>

namespace snark { namespace graphics { namespace plotting { namespace series {

struct config
{
    std::string chart;
    std::string color_name;
    QColor color;
    std::string name;
    bool scroll; // todo! a better name!
    std::string shape;
    std::string style;
    std::string title;
    float weight;
    
    config(): scroll( false ), weight( 1 ) {}
    config( const comma::command_line_options& options );
};

// todo: set pen etc on construction
class xy // todo? derive from base class? template on qt series type? time to decide: when introducing time series
{
    public:
        xy( QtCharts::QXYSeries* s = nullptr, const series::config& c = series::config() ): series_( s ), config_( c ) {}
        const series::config& config() const { return config_; }
        QtCharts::QXYSeries* operator()() { return series_; }
        const std::pair< QPointF, QPointF >& extents() const { return extents_; }
    
    private:
        QtCharts::QXYSeries* series_;
        series::config config_;
        std::pair< QPointF, QPointF > extents_;
};
    
} } } } // namespace snark { namespace graphics { namespace plotting { namespace series {
