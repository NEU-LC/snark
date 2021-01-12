// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <deque>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <QColor>
#include <QPointF>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QXYSeries>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/sync/synchronized.h>
#include "../../../graphics/block_buffer.h"
#include "point.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class series // todo: if series other than xyseries required, create series baseclass and derive from it xyseries, xyzseries, etc
{
    public:
        struct config_t
        {
            comma::csv::options csv;
            comma::uint32 size;
            std::string color_name;
            std::string shape;
            std::string style;
            float weight;
            QColor color;
            
            config_t() : size( 10000 ) {} // arbitrary
            config_t( const comma::command_line_options& options );
        };
        
        const config_t config;
        typedef std::deque< graphics::plotting::point > points_t;
        comma::synchronized< points_t > points; // quick and dirty

        template < typename S = QXYSeries >
        series( const config_t& config, QChart* chart );
        void start();
        bool is_shutdown() const;
        bool is_stdin() const;
        void shutdown();
        bool update();
        QtCharts::QXYSeries* series_todo; // todo: once stream phased out, rename series class to stream and this member to series
        QtCharts::QValueAxis* x_axis; // quick and dirty
        QtCharts::QValueAxis* y_axis; // quick and dirty

    protected:
        bool is_shutdown_;
        bool is_stdin_;
        comma::io::istream is_;
        comma::csv::input_stream< graphics::plotting::point > istream_;
        boost::scoped_ptr< boost::thread > thread_;
        comma::uint64 count_;
        bool has_x_;
        void read_();
        struct buffers_t_
        {
            block_buffer< QPointF > points;
            buffers_t_( comma::uint32 size );
            void add( const point& p );
            bool changed() const;
            void mark_seen();
        };
        buffers_t_ buffers_;
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
