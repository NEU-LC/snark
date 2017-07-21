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

#include "viewer.h"

namespace snark { namespace graphics { namespace view { namespace qt3d_v2 {

std::ostream& operator<<(std::ostream& os, const QVector3D& v)
{
    return os<<v.x()<<","<<v.y()<<","<<v.z();
}

viewer::viewer(controller_base* handler, const color_t& background_color, const qt3d::camera_options& camera_options, const QVector3D& scene_center, double arg_scene_radius,QMainWindow* parent) : 
    qt3d::gl::widget(camera_options,parent),
    handler(handler),
    scene_center(scene_center)
{
    scene_radius=arg_scene_radius;
    QTimer* timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( on_timeout() ) );
    timer->start( 40 );
    scene_radius_fixed_=false;
    scene_center_fixed_=false;
}
void viewer::reset_handler(controller_base* h){ handler=h; }
void viewer::init() { if(handler!=NULL) { handler->init(); } }
void viewer::on_timeout() { if(handler!=NULL) { handler->tick(); } }

void viewer::update_view(const QVector3D& min, const QVector3D& max)
{
    if( !scene_radius_fixed_ ) { scene_radius = 0.5 * ( max - min ).length(); }
    if( !scene_center_fixed_ ) { scene_center = 0.5 * ( min + max ); }
//     update the position of the far plane so that the full scene is displayed
    set_far_plane(4.6*scene_radius);
}

void viewer::look_at_center()
{
//     std::cerr<<"look_at_center "<<scene_center<<"; "<<scene_radius<<std::endl;
    camera.set_center(scene_center);
    camera.set_orientation(0, 3*M_PI/4, M_PI/4);
    if(camera_options_.orthographic)
    {
        size_=1.3*scene_radius;
    }
    else
    {
        camera.set_position(QVector3D(0,0,-2.6*scene_radius));
    }
}
void viewer::set_camera_position(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation)
{
    Eigen::Vector3d p = position - *m_offset;
    double cos_yaw = std::cos( orientation.z() );
    double sin_yaw = std::sin( orientation.z() );
    double sin_pitch = std::sin( orientation.y() );
    double cos_pitch = std::cos( orientation.y() );
    Eigen::Vector3d direction( cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch ); // todo: quick and dirty, forget about roll for now
    Eigen::Vector3d c = p + direction * 50; // quick and dirty
    scene_center=QVector3D(c.x(),c.y(),c.z());
    //to be tested
    camera.set_center(scene_center);    //set center
    camera.set_orientation(orientation.x(),M_PI-orientation.y(),-M_PI/2-orientation.z());
    camera.set_position(QVector3D(0,0,-p.norm()));    //camera is in 0,0,-z in world coordinate
}
    
} } } } // namespace snark { namespace graphics { namespace view { namespace qt3d_v2 {
    
