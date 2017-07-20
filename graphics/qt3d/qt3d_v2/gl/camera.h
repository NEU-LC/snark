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

#include <Eigen/Core>
#include <QMatrix4x4>

namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
struct camera_transform
{
    camera_transform(const QVector3D& up=QVector3D(0,0,-1),const QVector3D& center=QVector3D(),float z=-1);
    void pan(float dx,float dy);
    /// negative numbers push camera backward (zoom out)
    void zoom(float dz);
    /// rotate world on its own x and y axis
    /// apparent rotation of view point around pivot point
    void pivot(float dx,float dy);
    
    // sets world's center position
    void set_center(const QVector3D& v);
    /// sets world's orientation to euler angels
    void set_orientation(float roll,float pitch,float yaw);
    /// sets camera position in world coordinate
    /// z is distance to center and (x,y) component is pan
    void set_position(const QVector3D& v);
    QVector3D get_position() const;
    
    
    QMatrix4x4 world;
    QMatrix4x4 camera;
    QMatrix4x4 projection;
    QVector3D center;
    QVector3D up;   //not plugged in yet
};
    
} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
   
