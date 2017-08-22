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

#include "mesh_shader.h"
#include <QOpenGLPaintDevice>
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {
    

/// draw model
static const char *mesh_shader_source = R"(
    #version 150
    in vec3 vertex;
    in vec2 offset;
    in int mesh_index;
    out vec2 textCoord;
    uniform mat4 view_transform;
    uniform mat4 mesh_transform[100];
    void main() {
       gl_Position = transform_matrix*mesh_transform[mesh_index]*vec4(vertex,1) ;
       textCoord = offset;
    }
)";

static const char *mesh_fragment_source = R"(
    #version 150
    in vec2 textCoord;
    uniform sampler2D sampler;
    out highp vec4 frag_color;
    void main() {
       frag_color = texture2D(sampler,textCoord);
    }
)";

mesh_data::mesh_data() : vertices(NULL), normals(NULL), size(0), faces(NULL), faces_size(0) { }

//**********************************************************************************
mesh_shader::mesh_shader() { }
mesh_shader::~mesh_shader() { }
void mesh_shader::clear()
{
    for(auto& i : meshes) { i->destroy(); }
    meshes.clear();
}
void mesh_shader::init()
{
    std::cerr<<"mesh_shader::init "<<GL_MAX_UNIFORM_LOCATIONS<<" "<<GL_MAX_VERTEX_UNIFORM_COMPONENTS<<std::endl;
    initializeOpenGLFunctions();
    program.addShaderFromSourceCode( QOpenGLShader::Vertex, mesh_shader_source );
    program.addShaderFromSourceCode( QOpenGLShader::Fragment, mesh_fragment_source);
    program.bindAttributeLocation("vertex",0);
    program.bindAttributeLocation("offset",1);
    program.link();
    program.bind();
    transform_matrix_location=program.uniformLocation("transform_matrix");
//     sampler_location=program.uniformLocation("sampler");
    program.release();
    
    for(auto& j : meshes) { j->init(); }
}
void mesh_shader::paint(const QMatrix4x4& transform_matrix, const QSize& size)
{
    if(visible)
    {
        program.bind();
        program.setUniformValue(transform_matrix_location,transform_matrix);
//         program.setUniformValue(sampler_location,0);

        glEnable(GL_DEPTH_TEST);
        //?disable back-face culling
        //?disable depth test
        for(auto& j : meshes) { j->paint(); }
        program.release();
        glDisable(GL_DEPTH_TEST);
    }
}
void mesh_shader::destroy()
{
    for(auto& j : meshes) { j->destroy(); }
}
//**********************************************************************************
mesh::mesh() : visible(true) { }
mesh::~mesh() { }
void mesh::init()
{
    initializeOpenGLFunctions();

    vao.create();
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    vbo.bind();
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
//     glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( mesh_vertex ), reinterpret_cast<void *>( offsetof( mesh_vertex, position )));
//     glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( mesh_vertex ), reinterpret_cast<void *>( offsetof( mesh_vertex, offset )));
    vbo.release();
//     std::cerr<<"mesh::init"<<std::endl;
}
void mesh::update(const mesh_data& data)
{
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.bind();
    vbo.allocate(data.size*sizeof(Eigen::Vector3f));
    vbo.write(0,data.vertices,data.size*sizeof(Eigen::Vector3f));
    size=data.size;
    vbo.release();
}
void mesh::paint()
{
    if(visible)// && fbo)
    {
        glActiveTexture(GL_TEXTURE0);
//         glBindTexture(GL_TEXTURE_2D, fbo->mesh());
        
        QOpenGLVertexArrayObject::Binder binder(&vao);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glDrawArrays(GL_TRIANGLES,0,size);
//         static int debug=0;
//         if(debug++<10)
//             std::cerr<<"mesh::paint"<<std::endl;
    }
}
// void mesh::draw()
// {
//     if(fbo)
//     {
//         fbo->bind();
//         QOpenGLPaintDevice paint_dev(width, height);
//         QPainter painter(&paint_dev);
//         draw(painter);
//         painter.end();
//         fbo->release();
// //         std::cerr<<"mesh::draw"<<std::endl;
//     }
// }
void mesh::destroy()
{
//     fbo.release();
//     std::cerr<<"mesh::destroy"<<std::endl;
}

} } } // namespace snark { namespace graphics { namespace qopengl {
    
