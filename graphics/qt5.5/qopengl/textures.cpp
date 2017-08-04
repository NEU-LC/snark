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

/// @author Navid Pirmarzdashti

#include "textures.h"
#include <iostream>
#include "../../../math/rotation_matrix.h"

namespace snark { namespace graphics { namespace qopengl { namespace textures {

image::image(const QImage& qimage) : qimage(qimage), image_changed(true)
{
    
}
void image::update_quad(const Eigen::Vector3d& position,const Eigen::Vector3d& orientation,const Eigen::Vector2d& size)
{
    float x=position.x(), y=position.y(), z=position.z(), w=size.x(), h=size.y();
//     std::cerr<<"texture::update "<<x<<" "<<y<<" "<<z<<"; "<<w<<"x"<<h<<std::endl;
    quad.clear();
    quad.push_back(texture_vertex(x,y,z,0,0));
    quad.push_back(texture_vertex(x+w,y,z,1,0));
    quad.push_back(texture_vertex(x+w,y+h,z,1,1));
    quad.push_back(texture_vertex(x,y+h,z,0,1));
    //calculate rotation matrix
    Eigen::Matrix3f rotation=rotation_matrix(orientation).rotation().cast<float>();
    //apply to all points
    for(std::size_t i=0;i<quad.size();i++)
    {
        quad[i].position=rotation*quad[i].position;
    }
}
std::vector<texture_vertex> image::get_quad() const { return quad; }
void image::update()
{
    if(image_changed)
    {
        resize(qimage.size().width(),qimage.size().height());
        texture::draw();
        image_changed=false;
    }
    texture::update(&quad[0],quad.size());
}
void image::draw(QPainter& painter)
{
//     std::cerr<<"image::draw"<<std::endl;
    painter.drawImage(0,0,qimage);
}
    
} } } } // namespace snark { namespace graphics { namespace qopengl { namespace textures {
    
