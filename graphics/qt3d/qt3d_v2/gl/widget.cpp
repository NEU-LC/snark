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
#include <QPainter>
#include <math.h>
#include "widget.h"
#include "types.h"

namespace snark { namespace graphics { namespace qt3d { namespace gl {

widget::widget(const camera_options& camera_options, QWidget *parent )
    : QOpenGLWidget( parent ), program_( 0 ), camera_options_( camera_options ), size_( 0.4f )
{
}
widget::~widget()
{
    cleanup();
}

QSize widget::minimumSizeHint() const
{
    return QSize( 50, 50 );
}

QSize widget::sizeHint() const
{
    return QSize( 400, 400 );
}

void widget::cleanup()
{
    makeCurrent();
    for(auto& i : shapes) { i->destroy(); }
    if(program_)
    {
        delete program_;
        program_ = 0;
    }
    doneCurrent();
}

// If you want a good explanation of the projection, model and view matrices
// used in the shader code (model and view combined in the mv_matrix) see
// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-3-matrices/

// TODO: might be good (necessary) to make these shaders work with an older GLSL version. Perhaps 1.20.
static const char *vertex_shader_source = R"(
    #version 150
    in vec4 vertex;
    in vec4 color;
    in float point_size;
    out vec4 vert_color;
    uniform mat4 projection_matrix;
    uniform mat4 mv_matrix;
    void main() {
       vert_color = color;
       gl_Position = projection_matrix * mv_matrix * vertex;
       gl_PointSize=point_size;
    }
)";

static const char *fragment_shader_source = R"(
    #version 150
    in highp vec4 vert_color;
    out highp vec4 frag_color;
    void main() {
       frag_color = clamp( vert_color, 0.0, 1.0 );
    }
)";
void widget::begin_update()
{
    makeCurrent();
}
void widget::end_update()
{
    doneCurrent();
}

void widget::initializeGL()
{
    // In this example the widget's corresponding top-level window can change
    // several times during the widget's lifetime. Whenever this happens, the
    // QOpenGLWidget's associated context is destroyed and a new one is created.
    // Therefore we have to be prepared to clean up the resources on the
    // aboutToBeDestroyed() signal, instead of the destructor. The emission of
    // the signal will be followed by an invocation of initializeGL() where we
    // can recreate all resources.
    connect( context(), &QOpenGLContext::aboutToBeDestroyed, this, &widget::cleanup );

    initializeOpenGLFunctions();
    glClearColor( 0, 0, 0, 1 );
    program_ = new QOpenGLShaderProgram();
    program_->addShaderFromSourceCode( QOpenGLShader::Vertex, vertex_shader_source );
    program_->addShaderFromSourceCode( QOpenGLShader::Fragment, fragment_shader_source );
    program_->bindAttributeLocation( "vertex", 0 );
    program_->bindAttributeLocation( "color", 1 );
    program_->bindAttributeLocation("point_size",2);
    program_->link();

    program_->bind();
    projection_matrix_location_ = program_->uniformLocation( "projection_matrix" );
    mv_matrix_location_ = program_->uniformLocation( "mv_matrix" );

    for(auto& i : shapes) { i->init(); }
//     program_->release();

    // The camera always points along the z-axis. Pan moves the camera in x,y
    // coordinates and zoom moves in and out on the z-axis.
    // It starts at -1 because in OpenGL-land the transform is actually applied
    // to the world and the camera is stationary at 0,0,0.
    camera_.setToIdentity();
    camera_.translate( 0, 0, -1 );

    world_.setToIdentity();
    world_.translate( centre_of_rotation_ );
    init();
}

void widget::paintGL()
{
    QPainter painter( this );
    painter.beginNativePainting();

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_BLEND );
//     glEnable(GL_VERTEX_PROGRAM_POINT_SIZE); //use gl_PointSize from shader
//     glHint(GL_POINT_SMOOTH, GL_NICEST);
//     glEnable(GL_POINT_SMOOTH);  //circular point, otherwise draws square points
    glBlendEquation( GL_FUNC_ADD );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_CULL_FACE );

    
//     QOpenGLVertexArrayObject::Binder binder(&(shapes[0]->vao));
    program_->bind();
    program_->setUniformValue( projection_matrix_location_, projection_ );
    program_->setUniformValue( mv_matrix_location_, camera_ * world_ );

    for(auto& i : shapes) { i->paint(); }

    glDisable( GL_DEPTH_TEST );

    program_->release();

    painter.endNativePainting();

    painter.setPen( Qt::gray );
    painter.setFont( QFont( "Arial", 10 ));
    painter.drawText( rect(), Qt::AlignRight | Qt::AlignBottom
                    , QString("centre of rotation: %1 %2 %3").arg( centre_of_rotation_.x() )
                                                             .arg( centre_of_rotation_.y() )
                                                             .arg( centre_of_rotation_.z() ));

    painter.setPen( Qt::red );
    painter.setFont( QFont( "Arial", 10 ));
    painter.drawText( rect(), Qt::AlignHCenter | Qt::AlignTop
                    , QString("Warning: Qt3D v2 support is incomplete"));
}

void widget::set_projection()
{
    double aspect_ratio = (double) width() / height();
    projection_.setToIdentity();
    if( camera_options_.orthographic )
        projection_.ortho( -size_ * aspect_ratio, size_ * aspect_ratio, -size_, size_, 0.01f, 100.0f );
    else
        projection_.perspective( camera_options_.field_of_view, aspect_ratio, 0.01f, 100.0f );
}

void widget::resizeGL( int w, int h )
{
    set_projection();
}

void widget::mousePressEvent( QMouseEvent *event )
{
    last_pos_ = event->pos();
}

void widget::mouseMoveEvent( QMouseEvent *event )
{
    float dx = event->x() - last_pos_.x();
    float dy = event->y() - last_pos_.y();

    if ( event->buttons() & Qt::LeftButton )
    {
        world_.translate( centre_of_rotation_ );
        QMatrix4x4 inverted_world = world_.inverted();
        QVector4D x_axis = inverted_world * QVector4D( 1, 0, 0, 0 );
        QVector4D y_axis = inverted_world * QVector4D( 0, 1, 0, 0 );
        world_.rotate( dy, x_axis.toVector3D() );
        world_.rotate( dx, y_axis.toVector3D() );
        world_.translate( -centre_of_rotation_ );
        update();
    }
    else if ( event->buttons() & Qt::RightButton )
    {
        camera_.translate( dx / 500, -dy / 500, 0.0f );
        update();
    }
    last_pos_ = event->pos();
}

void widget::mouseDoubleClickEvent( QMouseEvent *event )
{
    if( event->button() == Qt::LeftButton )
    {
        boost::optional< QVector3D > point = viewport_to_3d( event->pos() );
        if( point ) { centre_of_rotation_ = *point; }
    }
}

void widget::wheelEvent( QWheelEvent *event )
{
    if( camera_options_.orthographic )
    {
        size_ *= ( 1 - 0.001 * event->delta() );
        set_projection();
    }
    else
    {
        camera_.translate( 0, 0, 0.01f * event->delta() );
    }
    update();
}

static const int pixel_search_width = 15;

// Take a 2d point on the viewport and determine the corresponding 3d point in space.
// The 2d point has to correspond to a pixel (or be close to one).
boost::optional< QVector3D > widget::viewport_to_3d( const QPoint& viewport_point )
{
    boost::optional< QVector3D > pixel = pixel_at_point( viewport_point, pixel_search_width );
    if( pixel )
    {
        QVector3D point_3d = pixel->unproject( camera_ * world_, projection_
                                             , QRect( 0, 0, width(), height() ));
        return point_3d;
    }
    else
    {
        return boost::optional< QVector3D >();
    }
}

// Take a viewport location and return the nearest active pixel within a square search area.
// Also converting from Qt coordinates (0,0 at top left) to OpenGL (0,0 at bottom left)
boost::optional< QVector3D > widget::pixel_at_point( const QPoint& viewport_point
                                                      , int search_width )
{
    std::vector< float > depth( search_width * search_width );
    // Convert the coordinates to openGL, offset to corner of search,
    // and clamp to limits to ensure we don't search outside the valid range.
    int x_min = std::min( std::max( viewport_point.x() - search_width / 2, 0 )
                        , width() - search_width );
    int y_min = std::min( std::max( height() - 1 - viewport_point.y() - search_width / 2, 0 )
                        , height() - search_width );
    makeCurrent();
    glReadPixels( x_min, y_min, search_width, search_width, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data() );
    doneCurrent();

    boost::optional< QVector3D > best_offset = pixel_nearest_centre( depth, search_width );
    if( best_offset ) { return *best_offset + QVector3D( x_min, y_min, 0 ); }
    else { return boost::optional< QVector3D >(); }
}

// Given a vector of pixel depths, representing a square search area,
// find the entry nearest the centre with a value less than 1.
// A value of 1 represents an empty pixel.
boost::optional< QVector3D > widget::pixel_nearest_centre( const std::vector< float >& depth
                                                            , int search_width )
{
    // distance represents the distance of the pixel from the centre of the search area
    int best_distance = std::numeric_limits<int>::max();
    boost::optional< QVector3D > best_offset;

    for( int j = 0; j < search_width; j++ )
    {
        for( int i = 0; i < search_width; i++ )
        {
            float pixel_depth = depth[ j * search_width + i ];
            if( pixel_depth < 1.0f )
            {
                int distance = abs( i - search_width / 2 ) + abs( j - search_width / 2 );
                if( distance < best_distance )
                {
                    best_offset = QVector3D( i, j, pixel_depth );
                    best_distance = distance;
                }
            }
        }
    }
    return best_offset;
}

} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
