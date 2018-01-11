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

#include "label_shader.h"
#include <QOpenGLPaintDevice>
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {

/*    image/texture shader
/// draw image from texture in 3d projection
static const char *texture_shader_source = R"(
    #version 130
    in vec3 vertex;
    in vec2 offset;
    out vec2 textCoord;
    uniform mat4 projection_matrix;
    void main() {
       gl_Position = projection_matrix* vec4(vertex,1) ;
       textCoord = offset;
    }
)";
static const char *texture_fragment_source = R"(
    #version 130
    in vec2 textCoord;
    uniform sampler2D sampler;
    out highp vec4 frag_color;
    void main() {
       frag_color = texture2D(sampler,textCoord);
    }
)";
*/

static const char *shader_source = R"(
    #version 130
    in vec3 vertex;
    in vec2 offset;
    in vec2 texture_size;
    out vec2 textCoord;
    uniform mat4 projection_matrix;
    uniform vec2 screen_size;
    uniform bool scaled;
    void main()
    {
        vec4 target=projection_matrix* vec4(vertex,1);
        if(scaled)
        {
            gl_Position = target + vec4(offset.x*texture_size.x/screen_size.x,offset.y*texture_size.y/screen_size.y,0,0);
        }
        else
        {
            gl_Position = vec4(target.x/target.z,target.y/target.z,0,1) + vec4(offset.x*texture_size.x/screen_size.x,offset.y*texture_size.y/screen_size.y,0,0);
        }
       textCoord = offset;
    }
)";

static const char *fragment_source = R"(
    #version 130
    in vec2 textCoord;
    uniform sampler2D sampler;
    out highp vec4 frag_color;
    void main()
    {
       frag_color = texture2D(sampler,textCoord);
    }
)";

label_vertex::label_vertex(float x,float y,float z,float ox, float oy,float w,float h) : position(x,y,z), offset(ox,oy), texture_size(w,h)
{
}

label_shader::label_shader() : visible(true), scaled(false)
{
//     labels.push_back(std::shared_ptr<label>(new label()));
    
}
label_shader::~label_shader()
{
    
}
void label_shader::clear()
{
    for(auto& i : labels) { i->destroy(); }
    labels.clear();
}
void label_shader::update()
{
    for(auto& i : labels) { i->update(); }
    
}
void label_shader::init()
{
//     std::cerr<<"label_shader::init"<<std::endl;
    initializeOpenGLFunctions();
    program.addShaderFromSourceCode( QOpenGLShader::Vertex, shader_source );
    program.addShaderFromSourceCode( QOpenGLShader::Fragment, fragment_source);
    program.bindAttributeLocation("vertex",0);
    program.bindAttributeLocation("offset",1);
    program.bindAttributeLocation("texture_size",2);
    program.link();
    program.bind();
    projection_matrix_location=program.uniformLocation("projection_matrix");
    sampler_location=program.uniformLocation("sampler");
    screen_size_location=program.uniformLocation("screen_size");
    scaled_location=program.uniformLocation("scaled");
    
    program.release();
    
    for(auto& j : labels) { j->init(); }
}
void label_shader::paint(const QMatrix4x4& projection_matrix, const QSize& size)
{
    if(visible)
    {
        program.bind();
        program.setUniformValue(projection_matrix_location,projection_matrix);
        program.setUniformValue(sampler_location,0);
        program.setUniformValue(screen_size_location,QVector2D(size.width()/2,size.height()/2));
        program.setUniformValue(scaled_location,scaled);
        
    //     static int counter=0;
    //     if(counter++<10)
    //         std::cerr<<"label_shader::paint "<<size.width()/2<<", "<<size.height()/2<<std::endl;
        
        //?disable back-face culling
        //?disable depth test
        for(auto& j : labels) { j->paint(); }
        program.release();
    }
}
void label_shader::destroy()   //destroy buffer
{
    for(auto& j : labels) { j->destroy(); }
}
label::label() : width(0),height(0) { } 
label::~label() { }
void label::init()
{
    initializeOpenGLFunctions();
    //shape init
    vao.create();
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    vbo.bind();
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( label_vertex ), reinterpret_cast<void *>( offsetof( label_vertex, position )));
    glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( label_vertex ), reinterpret_cast<void *>( offsetof( label_vertex, offset )));
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( label_vertex ), reinterpret_cast<void *>( offsetof( label_vertex, texture_size )));
    vbo.release();
}
void label::resize(int w,int h)
{
    if(width!=w || height!=h)
    {
        width=w;
        height=h;
//         std::cerr<<"label::resize "<<width<<","<<height<<std::endl;
        QOpenGLFramebufferObjectFormat format;
        format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        fbo.reset(new QOpenGLFramebufferObject(width, height, format));
    }
}
void label::update(float x,float y,float z)
{
//     std::cerr<<"label::update "<<width<<", "<<height<<std::endl;
    quad.clear();
    quad.push_back(label_vertex(x,y,z,0,0,width,height));
    quad.push_back(label_vertex(x,y,z,1,0,width,height));
    quad.push_back(label_vertex(x,y,z,1,1,width,height));
    quad.push_back(label_vertex(x,y,z,0,1,width,height));
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.bind();
    size_t size=quad.size()*sizeof(label_vertex);
    vbo.allocate(size);
    vbo.write(0,&quad[0],size);
    vbo.release();
}
void label::paint()
{
    if(fbo)
    {
//         static int counter=0;
//         if(counter++<2)
//             std::cerr<<"label_shader::paint"<<std::endl;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fbo->texture());
        
        QOpenGLVertexArrayObject::Binder binder(&vao);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glDrawArrays(GL_TRIANGLE_FAN,0,4);
    }
}

void label::draw()    //draw to texture
{
    if(fbo)
    {
//         std::cerr<<"label_shader::draw"<<std::endl;
        fbo->bind();
        
        QOpenGLPaintDevice paint_dev(width, height);
        QPainter painter(&paint_dev);
        draw(painter);
        painter.end();
        fbo->release();
    }
}
/*
void label::draw(QPainter& painter)
{
    painter.setFont(QFont("System",32));
//     painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
    painter.fillRect(0,0,width,height,Qt::green);
//         painter.setBackgroundMode(Qt::OpaqueMode);
//         painter.setBackground(QBrush(Qt::green));
    painter.setPen( Qt::blue );
    painter.drawText(QRect(2,4,width, height), Qt::AlignTop, "Boo!");
    painter.end();
//         QImage image2 = fbo->toImage();
//         image2.save("/home/navid/data/snark/view-points/bla4.png");
}
*/
void label::destroy()
{
    fbo.release();
}


} } } // namespace snark { namespace graphics { namespace qopengl {
    
