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

viewer::viewer(controller_base* handler, const color_t& background_color, const qt3d::camera_options& camera_options, const QVector3D& scene_center, double scene_radius,QMainWindow* parent) : 
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
    centre_of_rotation_=scene_center;
    world_.setToIdentity();
    world_.rotate(QQuaternion::fromEulerAngles(45,-180,-135));
    world_.translate(-centre_of_rotation_);
    camera_.setToIdentity();
    camera_.translate(0,0,-2.6*scene_radius);
}
void viewer::set_camera_position(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation)
{
    std::cerr<<"set_camera_position "<<position<<" "<<orientation<<std::endl;
    //to be tested
    world_.setToIdentity();
    world_.translate(centre_of_rotation_);
    world_.rotate(QQuaternion::fromEulerAngles(orientation.x(),orientation.y(),orientation.z()));
    world_.translate(-centre_of_rotation_);
    camera_.setToIdentity();
    camera_.translate(0,0,position.norm());
}
    
} } } } // namespace snark { namespace graphics { namespace view { namespace qt3d_v2 {
    
