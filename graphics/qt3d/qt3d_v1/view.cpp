// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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


/// @author Cedric Wohlleber

#include <QTimer>
#include <boost/array.hpp>
#include "../../../math/rotation_matrix.h"
#include "../camera_options.h"
#include "view.h"

namespace snark { namespace graphics { namespace qt3d {

/// constructor
/// @param background_color background color
/// @param fov camera field of view in degrees
/// @param z_up if true, the z-axis is pointing up, if false it is pointing down
/// @param orthographic use orthographic projection instead of perspective
view::view( const QColor4ub& background_color
          , const camera_options& camera_options
          , boost::optional< QVector3D > scene_center
          , boost::optional< double > scene_radius )
    : m_background_color( background_color )
    , m_sceneCenter( scene_center ? *scene_center : QVector3D( 0, 0, 0 ) )
    , scene_center_fixed_( scene_center )
    , m_z_up( camera_options.z_is_up )
    , scale_near_plane(false)
    , scene_radius_( scene_radius ? *scene_radius : 10 )
    , scene_radius_fixed_( scene_radius )
    , m_revolve( 0, 0, 0 )
    , m_show_coordinates( false )
{
    camera()->setFieldOfView( camera_options.field_of_view );
    camera()->setNearPlane( 0.001 );
    if( camera_options.orthographic )
    {
        camera()->setProjectionType( QGLCamera::Orthographic );
        camera()->setViewSize( QSize( scene_radius_, scene_radius_ ) );
    }
}

/// update the position of the far plane so that the full scene is displayed
void view::updateZFar()
{
    double zFar = ( camera()->eye() - camera()->center() ).length() + 2 * scene_radius_;
    camera()->setFarPlane( zFar );
    if(scale_near_plane)
    {
        double nearPlane=std::max(zFar-2.7*scene_radius_-10,0.01);
        camera()->setNearPlane(nearPlane);
    }
}

/// update the scene radius and center to display the region between @param min and @param max
void view::updateView( const QVector3D& min, const QVector3D& max )
{
    if( !scene_radius_fixed_ ) { scene_radius_ = 0.5 * ( max - min ).length(); }
    if( !scene_center_fixed_ ) { m_sceneCenter = 0.5 * ( min + max ); }
    updateZFar();
}

double view::scene_radius() const { return scene_radius_; }

/// setup the camera to look at the scene center
void view::lookAtCenter()
{
    double r = 1.5 * scene_radius_;
    double zEye;
    QVector3D upVector( 0, 0, 1 );
    if( m_z_up )
    {
        zEye = m_sceneCenter.z() + r;
    }
    else
    {
        zEye = m_sceneCenter.z() - r;
        upVector.setZ( -1 );
    }

    if( camera()->projectionType() == QGLCamera::Orthographic )
    {
        camera()->setViewSize( QSizeF( scene_radius_, scene_radius_ ) );
    }
    camera()->setCenter( m_sceneCenter );
    m_revolve = m_sceneCenter;
    camera()->setEye( QVector3D( m_sceneCenter.x() + r, m_sceneCenter.y() + r, zEye ) );
    QVector3D viewVector = camera()->center() - camera()->eye();
    QVector3D sideVector = QVector3D::crossProduct( viewVector, upVector );
    camera()->setUpVector( QVector3D::crossProduct( sideVector, viewVector ) );
}

/// draw a coordinate system centered on the scene rotation center
void view::draw_coordinates( QGLPainter *painter )
{
    if( m_coordinates && m_show_coordinates )
    {
        m_show_coordinates = false; // show once
        painter->modelViewMatrix().translate( m_revolve );
        m_coordinates->node()->draw( painter );
    }
}

/// handle mouse press
void view::mousePressEvent( QMouseEvent *e )
{
    if( e->button() == Qt::RightButton ) { m_startPan = e->pos(); }
    else if( e->button() == Qt::LeftButton ) { m_startRotate = e->pos(); }
    QGLView::mousePressEvent(e);
}

/// handle mouse release
void view::mouseReleaseEvent( QMouseEvent *e )
{
    if( e->button() == Qt::RightButton ) { m_startPan.reset(); }
    else if( e->button() == Qt::LeftButton ) { m_startRotate.reset(); }
    QGLView::mouseReleaseEvent(e);
}

/// returns a pseudo-distance from the point (x,y) to the unit circle
/// if the point is inside the circle, it is proportional to the euclidean distance to the circle
/// if the point is outside the circle, it is proportional to the inverse of the distance
static float projectOnCircle(float x, float y)
{
  const float size       = 1.0f;
  const float size2      = size*size;
  const float size_limit = size2*0.5;

  const float d = x*x + y*y;
  return d < size_limit ? sqrt(size2 - d) : size_limit/sqrt(d);
}

/// handle mouse move: pan or rotate the view ( using sphere rotation )
void view::mouseMoveEvent( QMouseEvent *e )
{
    if( m_startPan )
    {
        QPoint delta = e->pos() - *m_startPan;
        QPointF deltaF = delta;
        if( camera()->projectionType() == QGLCamera::Orthographic )
        {
            deltaF *= 1.5 * camera()->viewSize().width() / width();
        }
        else
        {
            // TODO adjusting the delta proportionally to width, can be improved
            double distance = ( camera()->eye() - camera()->center() ).length();
            if( distance > 5 )
            {
                deltaF *= 1.5 * distance / width();
            }
            else
            {
                // HACK for the case where the camera center has been moved by zooming in
                deltaF *= 0.5 * scene_radius_ / width();
            }
        }
        m_startPan = e->pos();
        QVector3D t = camera()->translation( deltaF.x(), -deltaF.y(), 0.0f );
        // TODO doesn't look like pure translation ?
        camera()->setEye( camera()->eye() - t );
        camera()->setCenter( camera()->center() - t );
    }
    else if( m_startRotate )
    {
        QPoint delta = e->pos() - *m_startRotate;
        if( delta.manhattanLength() > 0.1 )
        {
            m_show_coordinates = true;
            update();
            QTimer::singleShot( 1000, this, SLOT( hide_coordinates() ) );
        }

        qreal ratio = ( qreal ) width() / height();
        QMatrix4x4 world = camera()->projectionMatrix( ratio ) * camera()->modelViewMatrix();
        QVector3D center = world.map( m_revolve );
        // unprojected coordinates of the rotation center:
        double cx = 0.5 * (double)width() * ( center.x() + 1 );
        double cy = 0.5 * (double)height() * ( 1 - center.y() );

        // compute axis and angle transform between previous and current vectors on the sphere
        double previousX = ( (double)m_startRotate->x() - cx ) / width();
        double previousY = ( cy - (double)m_startRotate->y() ) / height();
        double currentX = ( (double)e->pos().x() - cx ) / width();
        double currentY = ( cy - (double)e->pos().y() ) / height();
        m_startRotate = e->pos();

        const QVector3D previous( previousX, previousY, projectOnCircle( previousX, previousY ) );
        const QVector3D current( currentX, currentY, projectOnCircle( currentX, currentY ) );
        const QVector3D axis = QVector3D::crossProduct( current, previous );
        const double sensitivity = 1.5;
        const double angle = sensitivity * 2.0 * std::asin( std::sqrt( axis.lengthSquared() / previous.lengthSquared() / current.lengthSquared() ) );

        // the axis is still in camera frame, convert it to world frame:
        QMatrix4x4 cameraMatrix = camera()->modelViewMatrix();
        Eigen::Matrix3d R;
        R << cameraMatrix( 0, 0 ) , cameraMatrix( 0, 1 ), cameraMatrix( 0, 2 ),
             cameraMatrix( 1, 0 ) , cameraMatrix( 1, 1 ), cameraMatrix( 1, 2 ),
             cameraMatrix( 2, 0 ) , cameraMatrix( 2, 1 ), cameraMatrix( 2, 2 );
        R.transposeInPlace();
        snark::rotation_matrix rotation( R );
        Eigen::Quaterniond cameraQ = rotation.quaternion();
        const QVector3D cameraAxis = QQuaternion( cameraQ.w(), cameraQ.x(), cameraQ.y(), cameraQ.z() ).rotatedVector( axis );

        QQuaternion q = QQuaternion::fromAxisAndAngle( cameraAxis, angle * 180 / M_PI );

        // rotate around scene center
        QVector3D centerVector = camera()->center() - m_revolve;
        QVector3D rotatedCenter = q.rotatedVector( centerVector ) + m_revolve;

        QVector3D eyeVector = camera()->eye() - m_revolve;
        QVector3D rotatedEye = q.rotatedVector( eyeVector ) + m_revolve;

        camera()->rotateCenter( q ); // to rotate the up vector
        camera()->setCenter( rotatedCenter );
        camera()->setEye( rotatedEye );
    }
}

/// handle mouse wheel: zoom in the scene
void view::wheelEvent( QWheelEvent *e )
{
    if( camera()->projectionType() == QGLCamera::Orthographic )
    {
        camera()->setViewSize( camera()->viewSize() * ( 1 - 0.001 * e->delta() ) );
    }
    else
    {
        QVector3D viewVector= camera()->eye() - camera()->center();
        qreal distance = viewVector.length();
        if( distance < 1 )
        {
            // HACK if camera center too close, push it further away
            camera()->setCenter( camera()->center() - 5 * viewVector.normalized() );
            distance = ( camera()->eye() - camera()->center() ).length();
        }
        const qreal coef = ( e->modifiers() & Qt::ShiftModifier ) ? (0.2 * scene_radius_) : qMax( distance, 0.2 * scene_radius_ );
        qreal zoomIncrement = -0.001 * e->delta() * coef;
        if ( !qFuzzyIsNull( zoomIncrement ) )
        {
            distance += zoomIncrement;
            if( distance < 0.1 )
            {
                distance = 0.1;
            }
            QRay3D viewLine( camera()->center(), viewVector.normalized() );
            camera()->setEye( viewLine.point( distance ) );
        }
    }
}

/// return 3d point in world coordinate from pixel coordinates and depth
QVector3D view::unproject( float x, float y, float depth )
{
    double mvmatrix[16];
    double projmatrix[16];
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev (GL_MODELVIEW_MATRIX, mvmatrix);
    glGetDoublev (GL_PROJECTION_MATRIX, projmatrix);
    // the following does the same as gluUnProject:
    QVector3D point = QVector3D( 2 * ( x - viewport[0] ) / viewport[2] - 1, 2 * ( height() - 1 - y - viewport[1] ) / viewport[3] - 1, 2 *depth - 1 );
    qreal ratio = ( qreal ) width() / height();
    QMatrix4x4 world = camera()->projectionMatrix( ratio ) * camera()->modelViewMatrix();
    QMatrix4x4 inverse = world.inverted();
    return inverse.map( point );
}

/// get 3d world point from clicked position
boost::optional< QVector3D > view::getPoint( const QPoint& point2d )
{
    float depth = 1;
    float x = point2d.x();
    float y = height() - 1 - point2d.y();
    // search for a point in the neighborhood to have more chance to hit
    boost::array< float, 18 > spiral = { { 0, 0, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0, -1, -1, 0, -1, 1, -1 } };
    for( unsigned int i = 0;  i < 9 && depth >= 1; i++ )
    {
        glReadPixels( x + spiral[ i << 1 ], y + spiral[ ( i << 1 ) + 1 ], 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    }
    if( depth < 1 )
    {
        return unproject( point2d.x(), point2d.y(), depth );
    }
    else
    {
        return boost::optional< QVector3D >();
    }
}

/// handle mouse double click: set rotation point
void view::mouseDoubleClickEvent( QMouseEvent *e )
{
    if( e->button() == Qt::RightButton ) { mouse_double_right_click_event( e ); }
    else if( e->button() == Qt::LeftButton )
    {
        boost::optional< QVector3D > point = getPoint( e->pos() );
        if( point )
        {
            m_revolve = *point;
            if( !m_coordinates )
            {
                m_coordinates = coordinates( 0.5, 0.1 );
            }
        }
        else
        {
            m_revolve = m_sceneCenter;
        }
        m_show_coordinates = true;
        update();
        QTimer::singleShot( 3000, this, SLOT( hide_coordinates() ) );
        QGLView::mouseDoubleClickEvent(e);
    }
}

} } } // namespace snark { namespace graphics { namespace qt3d {
