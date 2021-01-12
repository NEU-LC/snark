// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author Vsevolod Vlaskine

#ifndef SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_STREAM_H_
#define SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_STREAM_H_

#include <deque>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <QColor>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/sync/synchronized.h>
#include "../../../graphics/block_buffer.h"
#include "point.h"

class QwtPlot;

namespace snark { namespace graphics { namespace plotting {

class stream
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
        
        stream( const config_t& config );
        void start();
        bool is_shutdown() const;
        bool is_stdin() const;
        void shutdown();
        bool update();
        virtual void attach( QwtPlot* p ) = 0;
        virtual void update_plot_data() = 0;

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
            block_buffer< double > x;
            block_buffer< double > y;
            buffers_t_( comma::uint32 size );
            void add( const point& p );
            bool changed() const;
            void mark_seen();
        };
        buffers_t_ buffers_;
};
    
} } } // namespace snark { namespace graphics { namespace plotting {

#endif // #ifndef SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_STREAM_H_
