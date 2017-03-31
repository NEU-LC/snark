// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

#pragma once

#include "../../../qt3d/qt3d_v2/gl/widget.h"
#include "../types.h"
#include <QMainWindow>
#include <QTimer>
#include <iostream>

namespace snark { namespace graphics { namespace qt3d {
class camera_options;
} } }
typedef snark::graphics::qt3d::gl::color_t color_t;

namespace snark { namespace graphics { namespace view { namespace qt3d_v2 {

/**
 * redner and camera functions
 * qt3d v2 specific rednering, most functions are implemented in widget
 * this class implements interface used by controller
 */
class viewer : public qt3d::gl::widget
{
    Q_OBJECT
public:
    controller_base* handler;
    QVector3D scene_center;
    double scene_radius;
    bool scene_radius_fixed_;
    bool scene_center_fixed_;
public:
    viewer(controller_base* handler, const color_t& background_color, const qt3d::camera_options& camera_options, const QVector3D& scene_center, double scene_radius,QMainWindow* parent=NULL) : 
        qt3d::gl::widget(camera_options,parent),
        handler(handler),
        scene_center(scene_center),
        scene_radius(scene_radius)
    {
        QTimer* timer = new QTimer( this );
        connect( timer, SIGNAL( timeout() ), this, SLOT( on_timeout() ) );
        timer->start( 40 );
        scene_radius_fixed_=false;
        scene_center_fixed_=false;
    }
    void reset_handler(controller_base* h=NULL) { handler=h; }
protected:
    void init() { if(handler!=NULL) handler->init(); }
private slots:
    void on_timeout()
    {
        if(handler!=NULL)
            handler->tick();
    }
//     double scene_radius() const { return scene_radius_; }
public:
    void update_view( const QVector3D& min, const QVector3D& max )
    {
        if( !scene_radius_fixed_ ) { scene_radius = 0.5 * ( max - min ).length(); }
        if( !scene_center_fixed_ ) { scene_center = 0.5 * ( min + max ); }
//         updateZFar();
    }
    void look_at_center()
    {
        double r = 1.5 * scene_radius;
        QVector3D eye( scene_center.x() + r, scene_center.y() + r, scene_center.z() - r );
//         camera_.setToIdentity();
//         camera_.lookAt(eye,scene_center,QVector3D(0,0,-1));
    }
    boost::optional< Eigen::Vector3d > m_offset;
    void set_camera_position ( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation )
    {
        //to be implemented
    }

//from QGLView
//     QGLCamera * camera() const;
};
    

} } } } // namespace snark { namespace graphics { namespace view { namespace qt3d_v2 {

