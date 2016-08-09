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

#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include "gl_widget.h"
#include "types.h"

namespace snark { namespace graphics { namespace qt3d {

gl_widget::gl_widget( buffer_provider* buffer, QWidget *parent )
    : QOpenGLWidget( parent )
    , x_rot_( 0 )
    , y_rot_( 0 )
    , z_rot_( 0 )
    , buffer_( buffer )
    , program_( 0 )
{}

gl_widget::~gl_widget()
{
    cleanup();
}

QSize gl_widget::minimumSizeHint() const
{
    return QSize( 50, 50 );
}

QSize gl_widget::sizeHint() const
{
    return QSize( 400, 400 );
}

static void normalize_angle( int &angle )
{
    while( angle < 0 )        angle += 360 * 16;
    while( angle > 360 * 16 ) angle -= 360 * 16;
}

void gl_widget::set_x_rotation( int angle )
{
    normalize_angle( angle );
    if( angle != x_rot_ )
    {
        x_rot_ = angle;
        emit x_rotation_changed( angle );
        update();
    }
}

void gl_widget::set_y_rotation( int angle )
{
    normalize_angle( angle );
    if( angle != y_rot_ )
    {
        y_rot_ = angle;
        emit y_rotation_changed( angle );
        update();
    }
}

void gl_widget::set_z_rotation( int angle )
{
    normalize_angle( angle );
    if( angle != z_rot_ )
    {
        z_rot_ = angle;
        emit z_rotation_changed( angle );
        update();
    }
}

void gl_widget::cleanup()
{
    makeCurrent();
    vbo_.destroy();
    delete program_;
    program_ = 0;
    doneCurrent();
}

// If you want a good explanation of the projection, model and view matrices
// used in the shader code (model and view combined in the mv_matrix) see
// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-3-matrices/

static const char *vertex_shader_source =
    "#version 150\n"
    "in vec4 vertex;\n"
    "in vec4 color;\n"
    "out vec4 vert_color;\n"
    "uniform mat4 projection_matrix;\n"
    "uniform mat4 mv_matrix;\n"
    "void main() {\n"
    "   vert_color = color;\n"
    "   gl_Position = projection_matrix * mv_matrix * vertex;\n"
    "}\n";

// TODO: support alpha
static const char *fragment_shader_source =
    "#version 150\n"
    "in highp vec4 vert_color;\n"
    "out highp vec4 frag_color;\n"
    "void main() {\n"
    "   frag_color = clamp( vert_color, 0.0, 1.0 );\n"
    "}\n";

void gl_widget::initializeGL()
{
    // In this example the widget's corresponding top-level window can change
    // several times during the widget's lifetime. Whenever this happens, the
    // QOpenGLWidget's associated context is destroyed and a new one is created.
    // Therefore we have to be prepared to clean up the resources on the
    // aboutToBeDestroyed() signal, instead of the destructor. The emission of
    // the signal will be followed by an invocation of initializeGL() where we
    // can recreate all resources.
    connect( context(), &QOpenGLContext::aboutToBeDestroyed, this, &gl_widget::cleanup );

    initializeOpenGLFunctions();
    glClearColor( 0, 0, 0, 1 );

    program_ = new QOpenGLShaderProgram;
    program_->addShaderFromSourceCode( QOpenGLShader::Vertex, vertex_shader_source );
    program_->addShaderFromSourceCode( QOpenGLShader::Fragment, fragment_shader_source );
    program_->bindAttributeLocation( "vertex", 0 );
    program_->bindAttributeLocation( "color", 1 );
    program_->link();

    program_->bind();
    projection_matrix_location_ = program_->uniformLocation( "projection_matrix" );
    mv_matrix_location_ = program_->uniformLocation( "mv_matrix" );

    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    vao_.create();
    QOpenGLVertexArrayObject::Binder vaoBinder( &vao_ );

    // Setup our vertex buffer object.
    vbo_.create();
    vbo_.bind();
    vbo_.allocate( buffer_->buffer_data(), buffer_->buffer_size() * sizeof( vertex_t ));

    // Store the vertex attribute bindings for the program.
    setup_vertex_attribs();

    // Our camera never changes at the moment, we are moving the object
    camera_.setToIdentity();
    camera_.translate( 0, 0, -1 );

    program_->release();
}

void gl_widget::setup_vertex_attribs()
{
    vbo_.bind();
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray( 0 );
    f->glEnableVertexAttribArray( 1 );
    f->glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vertex_t ), reinterpret_cast<void *>( offsetof( vertex_t, position )));
    f->glVertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( vertex_t ), reinterpret_cast<void *>( offsetof( vertex_t, color )));
    vbo_.release();
}

void gl_widget::paintGL()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_CULL_FACE );

    world_.setToIdentity();
    world_.rotate( 180.0f - ( x_rot_ / 16.0f ), 1, 0, 0 );
    world_.rotate(            y_rot_ / 16.0f,   0, 1, 0 );
    world_.rotate(            z_rot_ / 16.0f,   0, 0, 1 );

    QOpenGLVertexArrayObject::Binder vaoBinder( &vao_ );
    program_->bind();
    program_->setUniformValue( projection_matrix_location_, projection_ );
    program_->setUniformValue( mv_matrix_location_, camera_ * world_ );

    glDrawArrays( GL_POINTS, 0, buffer_->buffer_size() );

    program_->release();
}

void gl_widget::resizeGL( int w, int h )
{
    projection_.setToIdentity();
    projection_.perspective( 45.0f, GLfloat( w ) / h, 0.01f, 100.0f );
}

void gl_widget::mousePressEvent( QMouseEvent *event )
{
    last_pos_ = event->pos();
}

void gl_widget::mouseMoveEvent( QMouseEvent *event )
{
    int dx = event->x() - last_pos_.x();
    int dy = event->y() - last_pos_.y();

    if ( event->buttons() & Qt::LeftButton ) {
        set_x_rotation( x_rot_ + 8 * dy );
        set_y_rotation( y_rot_ + 8 * dx );
    } else if ( event->buttons() & Qt::RightButton ) {
        set_x_rotation( x_rot_ + 8 * dy );
        set_z_rotation( z_rot_ + 8 * dx );
    }
    last_pos_ = event->pos();
}

} } } // namespace snark { namespace graphics { namespace qt3d {
