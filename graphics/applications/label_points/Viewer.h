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


#ifndef SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_VIEWER_H_
#define SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_VIEWER_H_

#ifdef WIN32
#include <WinSock2.h>
#define BOOST_FILESYSTEM_VERSION 3
#endif

#include <cmath>
#include <iostream>
#include <sstream>
#include <limits>
#include <math.h>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/optional.hpp>
#include <comma/string/string.h>

#include <snark/graphics/qt3d/view.h>

#include "./Dataset.h"
#include "./PointWithId.h"
#include "./Tools.h"

namespace snark { namespace graphics { namespace View {

static const long double pi_ = 3.14159265358979323846l;    
    
class Viewer : public qt3d::view
{
    Q_OBJECT

    public:
        Tools::Navigate navigate;
        Tools::PickId pickId;
        Tools::SelectPartition selectPartition;
        Tools::SelectId selectId;
        Tools::SelectClip selectClip; 
        Tools::Fill fill;

        Viewer( const std::vector< comma::csv::options >& options
              , bool labelDuplicated
              , const QColor4ub& background_color
              , bool orthographic = false, double fieldOfView = pi_ / 4 );

        void show( std::size_t i, bool visible ); // quick and dirty
        void setWritable( std::size_t i, bool writable ); // quick and dirty
        void save();
        void reload();
        const std::vector< boost::shared_ptr< Dataset > >& datasets() const;
        Dataset& dataset( std::size_t index ); // quick and dirty
        const Dataset& dataset( std::size_t index ) const; // quick and dirty

    public slots:
        void handleId( comma::uint32 id );

    signals:
        void initialized();

    protected:
        void saveStateToFile();
        void initializeGL( QGLPainter *painter );
        void paintGL( QGLPainter *painter );
        void mousePressEvent( QMouseEvent* e );
        void mouseReleaseEvent( QMouseEvent* e );
        void mouseMoveEvent( QMouseEvent* e );
        boost::optional< std::pair< Eigen::Vector3d, comma::uint32 > > pointSelection( const QPoint& point, bool writableOnly = false );

    private:
        friend class Tools::Tool; // quick and dirty
        friend class Tools::PickId; // quick and dirty
        friend struct Tools::SelectPartition; // quick and dirty
        friend struct Tools::SelectId; // quick and dirty
        friend class Tools::SelectClip; // quick and dirty
        friend struct Tools::Fill; // quick and dirty
        void setCamera();
                
        Tools::Tool* m_currentTool;
        std::vector< boost::shared_ptr< Dataset > > m_datasets;
        boost::optional< comma::uint32 > m_id;
        const QColor4ub m_background_color;
        std::vector< comma::csv::options > m_options;
        bool m_labelDuplicated;
        bool m_orthographic;
        double m_fieldOfView;
        boost::optional< QPoint > m_startPan;
        boost::optional< QPoint > m_startRotate;
        double m_sceneRadius;
};

} } } // namespace snark { namespace graphics { namespace View {

#endif // SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_VIEWER_H_
