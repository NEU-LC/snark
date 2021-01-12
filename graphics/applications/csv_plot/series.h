// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <deque>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <QColor>
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

class series
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
        series( const config_t& config );
        void start();
        bool is_shutdown() const;
        bool is_stdin() const;
        void shutdown();
        bool update();
        QtCharts::QXYSeries* operator()() { return series_; }
        const QtCharts::QXYSeries* operator()() const { return series_; }

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
            block_buffer< double > x; // todo: buffer value type
            block_buffer< double > y;
            buffers_t_( comma::uint32 size );
            void add( const point& p );
            bool changed() const;
            void mark_seen();
        };
        buffers_t_ buffers_;
        QtCharts::QXYSeries* series_;
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
