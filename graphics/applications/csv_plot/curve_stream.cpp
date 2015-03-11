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

#include <qwt/qwt_plot.h>
#include <comma/base/exception.h>
#include "curve_stream.h"

#include <iostream>

namespace snark { namespace graphics { namespace plotting {

static QwtPlotCurve::CurveStyle style_from_string( const std::string& s )
{
    if( s.empty() ) { return QwtPlotCurve::Lines; }
    if( s == "no-curve" ) { return QwtPlotCurve::NoCurve; }
    if( s == "lines" ) { return QwtPlotCurve::Lines; }
    if( s == "sticks" ) { return QwtPlotCurve::Sticks; }
    if( s == "steps" ) { return QwtPlotCurve::Steps; }
    if( s == "dots" || s == "points" ) { return QwtPlotCurve::Dots; }
    COMMA_THROW( comma::exception, "expected style, got \"" << s << "\"" );
}
    
curve_stream::curve_stream( const stream::config_t& config )
    : stream( config )
    , curve_( new QwtPlotCurve( &config.csv.filename[0] ) )
    , attached_( false )
{
    curve_->setPen( QPen( config.color, config.weight ) );
    curve_->setStyle( style_from_string( config.style ) );
}

void curve_stream::attach( QwtPlot* p )
{
    if( attached_ ) { COMMA_THROW( comma::exception, "already attached" ); }
    curve_->attach( p );
    attached_ = true;
}

void curve_stream::update_plot_data()
{
    curve_->setSamples( &buffers_.x.values()[0], &buffers_.y.values()[0], buffers_.x.size() );
}

} } } // namespace snark { namespace graphics { namespace plotting {
