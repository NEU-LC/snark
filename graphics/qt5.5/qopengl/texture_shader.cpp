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

#include "texture_shader.h"
#include <QOpenGLPaintDevice>
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {
    

/// draw image from texture in 3d projection
static const char *texture_shader_source = R"(
    #version 150
    in vec3 vertex;
    in vec2 offset;
    out vec2 textCoord;
    uniform mat4 transform_matrix;
    void main() {
       gl_Position = transform_matrix* vec4(vertex,1) ;
       textCoord = offset;
    }
)";
// frag_color = clamp( textCoord, 0.0, 1.0 );
// frag_color = texture2D(sampler,vec2(4,4));
//frag_color = clamp( sampler, 0.0, 1.0 );
    //uniform highp vec4 sampler;
static const char *texture_fragment_source = R"(
    #version 150
    in vec2 textCoord;
    uniform sampler2D sampler;
    out highp vec4 frag_color;
    void main() {
       frag_color = texture2D(sampler,textCoord);
    }
)";

texture_vertex::texture_vertex(float x,float y,float z,float ox,float oy) : position(x,y,z), offset(ox,oy) { }
//**********************************************************************************
texture_shader::texture_shader() { }
texture_shader::~texture_shader() { }
void texture_shader::clear()
{
    for(auto& i : textures) { i->destroy(); }
    textures.clear();
}
void texture_shader::update()
{
    for(auto& i : textures) { i->update(); }
}
void texture_shader::init()
{
    initializeOpenGLFunctions();
    program.addShaderFromSourceCode( QOpenGLShader::Vertex, texture_shader_source );
    program.addShaderFromSourceCode( QOpenGLShader::Fragment, texture_fragment_source);
    program.bindAttributeLocation("vertex",0);
    program.bindAttributeLocation("offset",1);
    program.link();
    program.bind();
    transform_matrix_location=program.uniformLocation("transform_matrix");
    sampler_location=program.uniformLocation("sampler");
    program.release();
    
    for(auto& j : textures) { j->init(); }
}
void texture_shader::paint(const QMatrix4x4& transform_matrix, const QSize& size)
{
    if(visible)
    {
        program.bind();
        program.setUniformValue(transform_matrix_location,transform_matrix);
        program.setUniformValue(sampler_location,0);

        glEnable(GL_DEPTH_TEST);
        //?disable back-face culling
        //?disable depth test
        for(auto& j : textures) { j->paint(); }
        program.release();
        glDisable(GL_DEPTH_TEST);
    }
}
void texture_shader::destroy()
{
    for(auto& j : textures) { j->destroy(); }
}
//**********************************************************************************
texture::texture() : visible(true), width(0), height(0) { }
texture::~texture() { }
void texture::init()
{
    initializeOpenGLFunctions();

    vao.create();
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    vbo.bind();
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( texture_vertex ), reinterpret_cast<void *>( offsetof( texture_vertex, position )));
    glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( texture_vertex ), reinterpret_cast<void *>( offsetof( texture_vertex, offset )));
    vbo.release();
//     std::cerr<<"texture::init"<<std::endl;
}
void texture::resize(int w,int h)
{
    if(width!=w || height!=h)
    {
        width=w;
        height=h;
        QOpenGLFramebufferObjectFormat format;
        format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        fbo.reset(new QOpenGLFramebufferObject(width, height, format));
//         std::cerr<<"texture::resize "<<width<<" "<<height<<std::endl;
    }
}
void texture::update(const texture_vertex* vertices, std::size_t size)
{
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.bind();
    vbo.allocate(size*sizeof(texture_vertex));
    vbo.write(0,vertices,size*sizeof(texture_vertex));
    vbo.release();
}
void texture::paint()
{
    if(visible && fbo)
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fbo->texture());
        
        QOpenGLVertexArrayObject::Binder binder(&vao);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glDrawArrays(GL_TRIANGLE_FAN,0,4);
//         static int debug=0;
//         if(debug++<10)
//             std::cerr<<"texture::paint"<<std::endl;
    }
}
void texture::draw()
{
    if(fbo)
    {
        fbo->bind();
        QOpenGLPaintDevice paint_dev(width, height);
        QPainter painter(&paint_dev);
        draw(painter);
        painter.end();
        fbo->release();
//         std::cerr<<"texture::draw"<<std::endl;
    }
}
void texture::destroy()
{
    fbo.release();
//     std::cerr<<"texture::destroy"<<std::endl;
}

} } } // namespace snark { namespace graphics { namespace qopengl {
