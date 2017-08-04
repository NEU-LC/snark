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

#pragma once

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFramebufferObject>
#include <vector>
#include <memory>
#include <QPainter>
#include <Eigen/Core>

namespace snark { namespace graphics { namespace qopengl {

/// texture vertex has a 3d point and its normalized texture position
struct texture_vertex
{
    /// x,y,z of the corner point 
    Eigen::Vector3f position;
    /// normalized offset of other corners 0/1
    Eigen::Vector2f offset;
    texture_vertex(float x,float y,float z,float ox,float oy);
};

class texture : protected QOpenGLFunctions
{
    friend class texture_shader;
public:
    texture();
    virtual ~texture();
    
    /// set to false to hide it
    bool visible;
    
protected:
    /// update vertices
    void update(const texture_vertex* vertices, std::size_t size);
    /// resize texture buffer
    /// width and height in pixels
    void resize(int width,int height);

protected:
    /// update vertices and texture buffer
    virtual void update()=0;
    /// draw the label
    virtual void draw(QPainter& painter)=0;
    
    //no need to override these
    virtual void init();
    virtual void paint();
    void draw();
    virtual void destroy();
    
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    
    std::unique_ptr<QOpenGLFramebufferObject> fbo;
    int width;
    int height;
};

class texture_shader : protected QOpenGLFunctions
{
    friend class widget;
public:
    texture_shader();
    virtual ~texture_shader();
    void clear();   //delete labels
    void update();  //init and update all added labels
    
public:
    std::vector<std::shared_ptr<texture>> textures;
    bool visible;

protected:
    //GL context should be set and voa bound for these functions by caller (i.e. gl_widget)
    virtual void init();    //create texture buffer
    virtual void paint(const QMatrix4x4& transform_matrix, const QSize& size);  //invoke glDraw*
    virtual void destroy();   //destroy buffer
protected:
    
    QOpenGLShaderProgram program;
    int transform_matrix_location;
    int sampler_location;
//     int screen_size_location;
};

} } } // namespace snark { namespace graphics { namespace qopengl {
