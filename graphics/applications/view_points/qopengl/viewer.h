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

#include "../../../qt5.5/qopengl/widget.h"
#include "../types.h"
#include <QMainWindow>
#include <QTimer>

namespace snark { namespace graphics { namespace qt3d {
class camera_options;
} } }
typedef snark::graphics::qopengl::color_t color_t;

namespace snark { namespace graphics { namespace view { namespace qopengl {

/**
 * redner and camera functions
 * qt3d v2 specific rednering, most functions are implemented in widget
 * this class implements interface used by controller
 */
class viewer : public snark::graphics::qopengl::widget
{
    Q_OBJECT
public:
    controller_base* handler;
    QVector3D scene_center;
    bool scene_radius_fixed;
    bool scene_center_fixed;
    
public:
    viewer(controller_base* handler, const color_t& background_color, const qt3d::camera_options& camera_options, 
           const QVector3D& scene_center, double scene_radius,QMainWindow* parent=NULL);
    void reset_handler(controller_base* h=NULL);
    
protected:
    void init();
    void double_right_click(const boost::optional<QVector3D>& point);
    
private slots:
    void on_timeout();
    
public:
    void update_view(const QVector3D& min, const QVector3D& max);
    void look_at_center();
    boost::optional< Eigen::Vector3d > m_offset;
    void set_camera_position(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation);
    bool stdout_allowed;

//     double scene_radius() const { return scene_radius_; }
    
//from QGLView
//     QGLCamera * camera() const;
};
    

} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {

