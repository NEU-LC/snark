// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#include "types.h"
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>

namespace snark { namespace graphics { namespace qopengl {

// TODO refactor shape shader into separate class out of widget, similar to label_shader
// class shape_shader
// {
// public:
//     std::vector<std::shared_ptr<shape>> shapes;
//     //set GL context and bind voa, ?optionaly synchronize with paint
//     virtual void begin_update()=0;
//     virtual void end_update()=0;
// protected:
//     void init();
//     void paint();
//     void destroy();
// };

class shape : protected QOpenGLFunctions
{
protected:
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    friend class widget;
public:
    shape(GLenum mode=GL_POINTS);
    virtual ~shape();
    //this can only be called between begin_update and end_update
    //size: size of array e.g. update(v.data(),v.size())
    virtual void update(const vertex_t* data, std::size_t size);    //write data to internal buffer
public:
    /// set true to draw, false to hide; call widget update to take effect
    bool visible;
protected:
    //GL context should be set and voa bound for these functions by caller (i.e. gl_widget)
    virtual void init();    //create buffer
    virtual void paint();  //invoke glDraw*
    virtual void destroy();   //destroy buffer
    GLenum mode;
    std::size_t size_;
private:
};

namespace shapes {
    
class point : public shape
{
public:
    float point_size;
    point(float point_size=1);
protected:
    void paint();
};

/// Two vertices per line. Vertices 0 and 1 are considered a line. Vertices 2 and 3 are considered a line, etc.
class lines : public shape
{
    float line_width;
public:
    lines(float line_width=1);
protected:
    void paint();
};

/// The adjacent vertices are considered lines. Vertice 0 and 1; then vertices 1 and 2, etc.
class line_strip : public shape
{
public:
    line_strip();
};

/// The adjacent vertices are considered lines (similar to line_strip) and then last vertice and first to close the loop
class line_loop : public shape
{
public:
    line_loop();
};

class triangles : public shape
{
public:
    bool fill;
    triangles(bool fill=false);
protected:
    void paint();
};

} // namespace shapes {

} } } // namespace snark { namespace graphics { namespace qopengl {
    
