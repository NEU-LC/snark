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


#ifndef SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_TOOLS_H_
#define SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_TOOLS_H_

#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <qcursor.h>
#include <qevent.h>
#include <qobject.h>
#include <comma/base/types.h>
#include <snark/math/interval.h>
#include <Eigen/Core>
#include <Qt3D/qcolor4ub.h>
#include <Qt3D/qglpainter.h>
#include "Icons.h"

namespace snark { namespace graphics { namespace View { class Viewer; } } }

namespace snark { namespace graphics { namespace View { namespace Tools {

QColor4ub colorFromId( comma::uint32 id );

class Tool : public QObject
{   
    public:
        Tool( Viewer& viewer, QCursor* cursor );
        virtual ~Tool() {}
        void toggle( bool checked );
        virtual void onMousePress( QMouseEvent* e );
        virtual void onMouseRelease( QMouseEvent* e );
        virtual void onMouseMove( QMouseEvent* e );
        virtual void draw( QGLPainter* painter );
    
    protected:
        Viewer& m_viewer;
        boost::scoped_ptr< QCursor > m_cursor;
        virtual void init() {}
        virtual void reset() {}
};

struct Navigate : public Tool
{
    Navigate( Viewer& viewer );
};

class PickId : public Tool
{
    Q_OBJECT
    
    public:
        PickId( Viewer& viewer );
        void onMousePress( QMouseEvent* e );
        void shakeColors();
    
    signals:
        void valueChanged( comma::uint32 id );
};

struct Fill : public Tool
{
    Fill( Viewer& viewer );
    void onMousePress( QMouseEvent* e );
};

struct SelectPartition : public Tool
{
    SelectPartition( Viewer& viewer );
    void onMousePress( QMouseEvent* e );
    void onMouseRelease( QMouseEvent* e );
    void onMouseMove( QMouseEvent* e );
};

struct SelectId : public Tool
{
    SelectId( Viewer& viewer );
    void onMousePress( QMouseEvent* e );
    void onMouseRelease( QMouseEvent* e );
    void onMouseMove( QMouseEvent* e );
};

class SelectClip : public Tool
{
    public:
        SelectClip( Viewer& viewer );
        void onMousePress( QMouseEvent* e );
        void onMouseRelease( QMouseEvent* e );
        void onMouseMove( QMouseEvent* e );
        void draw( QGLPainter* painter );
        
    private:
        boost::optional< QRect > m_rectangle;
        Eigen::Vector3d m_center;
        Eigen::Vector3d m_radius;
};

} } } } // namespace snark { namespace graphics { namespace View { namespace Tools {

#endif // SNARK_GRAPHICS_APPLICATIONS_LABELPOINTS_TOOLS_H_
