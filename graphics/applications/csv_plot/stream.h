// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <deque>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <QColor>
#include <QPointF>
#include <QtCharts/QChart>
#include <QtCharts/QXYSeries>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/sync/synchronized.h>
#include "../../../graphics/block_buffer.h"
#include "point.h"
#include "series.h"

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class stream // todo: if stream other than xy stream required, create stream baseclass and derive from it xy stream, xyz stream, etc
{
    public:
        struct config_t
        {
            comma::csv::options csv;
            bool pass_through;
            plotting::series::config series;
            comma::uint32 size;
            
            config_t() : pass_through( false ), size( 10000 ) {}
            config_t( const comma::command_line_options& options );
        };
        
        QtCharts::QXYSeries* series; // todo: array of series supporting multiple feeds from the same stream, potentially targeting multiple charts
        const config_t config;
        typedef std::deque< graphics::plotting::record > records_t;
        comma::synchronized< records_t > records; // quick and dirty

        stream( QXYSeries* s, const config_t& config );
        void start();
        bool is_shutdown() const;
        bool is_stdin() const;
        void shutdown();
        bool update();
        std::pair< QPointF, QPointF > extents() const { return extents_; }
        unsigned int size() const { return size_; }

    protected:
        bool is_shutdown_;
        bool is_stdin_;
        comma::io::istream is_;
        comma::csv::input_stream< graphics::plotting::record > istream_;
        boost::scoped_ptr< comma::csv::passed< graphics::plotting::record > > passed_; // boost::optional< comma::csv::passed< graphics::plotting::record > > passed_; // todo: move semantics
        boost::scoped_ptr< boost::thread > thread_;
        comma::uint64 count_;
        bool has_x_;
        void read_();
        struct buffers_t_
        {
            block_buffer< plotting::record > records;
            buffers_t_( comma::uint32 size );
            void add( const record& p );
            bool changed() const;
            void mark_seen();
        };
        buffers_t_ buffers_;
        unsigned int size_;
        std::pair< QPointF, QPointF > extents_; // todo: should be a pair of min record and max record
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
