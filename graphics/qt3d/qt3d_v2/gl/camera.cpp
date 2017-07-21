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

#include "camera.h"

namespace snark { namespace graphics { namespace qt3d { namespace gl {

camera_transform::camera_transform(const QVector3D& up,const QVector3D& c,float z) : center(c), up(up)
{
    // The camera always points along the z-axis. Pan moves the camera in x,y
    // coordinates and zoom moves in and out on the z-axis.
    // It starts at -1 because in OpenGL-land the transform is actually applied
    // to the world and the camera is stationary at 0,0,0.
    camera.setToIdentity();
    camera.translate(0, 0, -1);
    world.setToIdentity();
    world.translate(-center);
}
void camera_transform::pan(float dx,float dy)
{
    camera.translate(dx,dy,0);
}
void camera_transform::zoom(float dz)
{
    camera.translate(0,0,dz);
}
void camera_transform::pivot(float dx,float dy)
{
    world.translate(center);
    QMatrix4x4 inverted_world = world.inverted();
    QVector4D x_axis = inverted_world * QVector4D(1, 0, 0, 0);
    QVector4D y_axis = inverted_world * QVector4D(0, 1, 0, 0);
    world.rotate(dy, x_axis.toVector3D());
    world.rotate(dx, y_axis.toVector3D());
    world.translate(-center);
}
void camera_transform::set_center(const QVector3D& v)
{
//     world.translate(center);
    center=v;
//     world.translate(-center);
}
void camera_transform::set_orientation(float roll,float pitch,float yaw)
{
    world.setToIdentity();
    world.rotate(QQuaternion::fromEulerAngles(pitch*180/M_PI,roll*180/M_PI,yaw*180/M_PI));
    world.translate(-center);
}
void camera_transform::set_position(const QVector3D& v)
{
    camera.setToIdentity();
    camera.translate(v);
}
QVector3D camera_transform::get_position() const
{
    return camera.column(3).toVector3DAffine();
}
    
} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
