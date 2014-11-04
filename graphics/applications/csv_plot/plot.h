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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#ifndef SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_PLOT_H_
#define SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_PLOT_H_

#include <qwt/qwt_plot.h>
#include <boost/scoped_ptr.hpp>

namespace snark { namespace graphics {

class plot : public QwtPlot
{
public:
  plot( QWidget *parent=0, char *name=0 ) : QwtPlot( parent )
  {
    // Show a title
    setTitle( "This is an Example" );

//     // Show a legend at the bottom
//     setAutoLegend( true );
//     setLegendPos( Qwt::Bottom );
// 
//     // Show the axes
//     setAxisTitle( xBottom, "x" );
//     setAxisTitle( yLeft, "y" );
// 
//     // Insert two curves and get IDs for them
//     long cSin = insertCurve( "y = sin(x)" );
//     long cSign = insertCurve( "y = sign(sin(x))" );
// 
//     // Calculate the data, 500 points each
//     const int points = 500;
//     double x[ points ];
//     double sn[ points ];
//     double sg[ points ];
// 
//     for( int i=0; i<points; i++ )
//     {
//       x[i] = (3.0*3.14/double(points))*double(i);
//     
//       sn[i] = 2.0*sin( x[i] );
//       sg[i] = (sn[i]>0)?1:((sn[i]<0)?-1:0);
//     }
// 
//     // Copy the data to the plot
//     setCurveData( cSin, x, sn, points );
//     setCurveData( cSign, x, sg, points );
// 
//     // Set the style of the curves
//     setCurvePen( cSin, QPen( blue ) );
//     setCurvePen( cSign, QPen( green, 3 ) );

    // Show the plots
    replot();
  }
};
    
} } // namespace snark { namespace graphics {

#endif // #ifndef SNARK_GRAPHICS_APPLICATIONS_CSV_PLOT_PLOT_H_
