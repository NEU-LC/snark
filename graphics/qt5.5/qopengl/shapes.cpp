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

#include "shapes.h"
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {
    
shape::shape(GLenum mode):mode(mode),size_(0)
{
    
}
shape::~shape()
{
}
void shape::init()
{
    initializeOpenGLFunctions();
    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    vao.create();
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    vbo.bind();
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vertex_t ), reinterpret_cast<void *>( offsetof( vertex_t, position )));
    glVertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( vertex_t ), reinterpret_cast<void *>( offsetof( vertex_t, color )));
    vbo.release();
}


void shape::update(const vertex_t* data, std::size_t size)
{
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.bind();
    vbo.allocate(size*sizeof(vertex_t));
    vbo.write(0,data,size*sizeof(vertex_t));
    vbo.release();
    size_=size;
}
void shape::paint()
{
    QOpenGLVertexArrayObject::Binder binder(&vao);
    glDrawArrays(mode,0,size_);
}
void shape::destroy()
{
    vbo.destroy();
}
namespace shapes {
    
point::point(float point_size) : shape(GL_POINTS),point_size(point_size)
{
}
void point::paint()
{
    //disable GL_PROGRAM_POINT_SIZE
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    if(point_size!=1) { glEnable(GL_POINT_SMOOTH); } //circular point, otherwise draws square points
    glPointSize(point_size);
    shape::paint();
}

lines::lines(float line_width) : shape(GL_LINES),line_width(line_width) { }
void lines::paint()
{
    if(line_width!=1)
    {
//         std::cerr<<"lines::paint"
        glLineWidth(line_width);
        glEnable(GL_LINE_SMOOTH);
    }
    shape::paint();

// //if not supported by vga
// GLfloat lineWidthRange[2];
// glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lineWidthRange);
}

line_strip::line_strip(float line_width) : shape(GL_LINE_STRIP),line_width(line_width) { }
void line_strip::paint()
{
    if(line_width!=1)
    {
        glLineWidth(line_width);
        glEnable(GL_LINE_SMOOTH);
    }
    shape::paint();
}

line_loop::line_loop(float line_width) : shape(GL_LINE_LOOP),line_width(line_width) { }
void line_loop::paint()
{
    if(line_width!=1)
    {
        glLineWidth(line_width);
        glEnable(GL_LINE_SMOOTH);
    }
    shape::paint();
}

triangles::triangles(bool fill) : shape(GL_TRIANGLES),fill(fill) { }
void triangles::paint()
{
    glPolygonMode(GL_FRONT_AND_BACK,fill ? GL_FILL : GL_LINE);
    shape::paint();
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
}

} // namespace shapes {

} } } // namespace snark { namespace graphics { namespace qopengl {
    
