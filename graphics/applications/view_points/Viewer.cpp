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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


/// @author Vsevolod Vlaskine, Cedric Wohlleber

#ifndef WIN32
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#else
#include <signal.h>
#endif
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/thread_time.hpp>
#include <comma/csv/stream.h>
#include <comma/name_value/ptree.h>
#include <comma/visiting/apply.h>
#include "./Viewer.h"
#include <QTimer>

namespace snark { namespace graphics { namespace View {

Viewer::camera_position_output::camera_position_output( const Viewer& viewer ) : viewer_( viewer ) {}

void Viewer::camera_position_output::write()
{
    boost::property_tree::ptree p;
    comma::to_ptree to_ptree( p );
    comma::visiting::apply( to_ptree ).to( *viewer_.camera() );
    boost::property_tree::write_json( std::cout, p );
}

Viewer::Viewer( const QColor4ub& background_color
              , double fov
              , bool z_up
              , bool orthographic
              , boost::optional< comma::csv::options > camera_csv, boost::optional< Eigen::Vector3d > cameraposition
              , boost::optional< Eigen::Vector3d > cameraorientation
              , boost::property_tree::ptree* camera_config
              , boost::optional< Eigen::Vector3d > scene_center
              , boost::optional< double > scene_radius
              , bool output_camera_position )
    : qt3d::view( background_color
                , fov
                , z_up
                , orthographic
                , scene_center ? boost::optional< QVector3D >( QVector3D( scene_center->x()
                                                                        , scene_center->y()
                                                                        , scene_center->z() ) )
                               : boost::optional< QVector3D >()
                , scene_radius )
    , m_lookAt( false )
    , m_cameraposition( cameraposition )
    , m_cameraorientation( cameraorientation )
{
    if( output_camera_position ) { camera_position_output_.reset( new camera_position_output( *this ) ); }
    QTimer* timer = new QTimer( this );
    timer->start( 40 );
    connect( timer, SIGNAL( timeout() ), this, SLOT( read() ) );
    if( camera_csv ) { m_cameraReader.reset( new CameraReader( *camera_csv ) ); }
    if( camera_config )
    {
        comma::from_ptree from_ptree( *camera_config, true );
        comma::visiting::apply( from_ptree ).to( *camera() );
        //boost::property_tree::ptree p;
        //comma::to_ptree to_ptree( p );
        //comma::visiting::apply( to_ptree ).to( *camera() );
        //std::cerr << "view-points: camera set to:" << std::endl;
        //boost::property_tree::write_json( std::cerr, p );
    }
    m_cameraFixed = m_cameraposition || m_cameraReader || camera_config;
}

void Viewer::shutdown()
{
    m_shutdown = true;
    #ifdef WIN32
    ::raise( SIGTERM ); // according to windows docs, SIGINT does not work on windows
    #else // #ifdef WIN32
    ::raise( SIGINT ); // quick and dirty... or maybe not so dirty: to interrupt blocking read() in reader threads, if closed from qt window
    #endif // #ifdef WIN32
    if( m_cameraReader ) { m_cameraReader->shutdown(); }
    for( unsigned int i = 0; i < readers.size(); ++i ) { readers[i]->shutdown(); }
}

void Viewer::initializeGL( QGLPainter *painter )
{
    (void) painter;
    glEnable(GL_BLEND);
//     glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    qglClearColor( m_background_color.toColor() );
    if( m_cameraReader ) { m_cameraReader->start(); }
    for( unsigned int i = 0; i < readers.size(); ++i ) { readers[i]->start(); }
}

void Viewer::read()
{
    for( unsigned int i = 0; !m_offset && i < readers.size(); ++i )
    {
        if( readers[i]->empty() ) { continue; }
        Eigen::Vector3d p = readers[i]->somePoint();
        m_offset = std::fabs( p.x() ) > 1000 || std::fabs( p.y() ) > 1000 || std::fabs( p.z() ) > 1000 ? p : Eigen::Vector3d( 0, 0, 0 );
        std::cerr << "view-points: reader no. " << i << " scene offset (" << m_offset->transpose() << "); scene radius: " << scene_radius() << std::endl;
    }
    if( !m_offset ) { return; }
    for( unsigned int i = 0; i < readers.size(); ++i )
    {
        readers[i]->update( *m_offset );
    }
    m_shutdown = true;
    bool ready_to_look = true;
    for( unsigned int i = 0; m_shutdown && i < readers.size(); ++i )
    {
        m_shutdown = m_shutdown && readers[i]->isShutdown();
        ready_to_look = ready_to_look && ( readers[i]->isShutdown() || ( readers.size() > 1 && readers[i]->isStdIn() ) );
    }
    if( !m_cameraReader && m_cameraposition )
    {
        setCameraPosition( *m_cameraposition, *m_cameraorientation );
        m_cameraposition.reset();
        m_cameraorientation.reset();
    }
    else if( m_cameraReader )
    {
        Eigen::Vector3d position = m_cameraReader->position();
        Eigen::Vector3d orientation = m_cameraReader->orientation();
        if( !m_cameraposition || !m_cameraposition->isApprox( position ) || !m_cameraorientation->isApprox( orientation ) )
        {
            m_cameraposition = position;
            m_cameraorientation = orientation;
            setCameraPosition( position, orientation );
        }
    }
    else if( readers[0]->m_extents && readers[0]->m_num_points > 0 && ( m_shutdown || ready_to_look || readers[0]->m_num_points >= readers[0]->size / 10 ) )
    {
        QVector3D min( readers[0]->m_extents->min().x(), readers[0]->m_extents->min().y(), readers[0]->m_extents->min().z() );
        QVector3D max( readers[0]->m_extents->max().x(), readers[0]->m_extents->max().y(), readers[0]->m_extents->max().z() );
        updateView( min, max );
        if( !m_cameraFixed && !m_lookAt )
        {
            m_lookAt = true;
            lookAtCenter();
        }
    }
    if( !m_shutdown ) { update(); }
}

void Viewer::paintGL( QGLPainter *painter )
{
    for( unsigned int i = 0; i < readers.size(); ++i )
    {
        if( !readers[i]->show() ) { continue; }
        if( readers[i]->pointSize > 1 ) { ::glEnable( GL_POINT_SMOOTH ); }
        ::glPointSize( readers[i]->pointSize );
        readers[i]->render( painter );
        if( readers[i]->pointSize > 1 ) { ::glDisable( GL_POINT_SMOOTH ); }
    }
    draw_coordinates( painter );
    if( camera_position_output_ ) { camera_position_output_->write(); }
}

void Viewer::setCameraPosition ( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation )
{
    Eigen::Vector3d p = position - *m_offset;
    camera()->setUpVector( QVector3D( 0, 0, m_z_up ? 1 : -1 ) );
    camera()->setEye( QVector3D( p.x(), p.y(), p.z() ) );
    double cos_yaw = std::cos( orientation.z() );
    double sin_yaw = std::sin( orientation.z() );
    double sin_pitch = std::sin( orientation.y() );
    double cos_pitch = std::cos( orientation.y() );
    Eigen::Vector3d direction( cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch ); // todo: quick and dirty, forget about roll for now
    Eigen::Vector3d scene_center = p + direction * 50; // quick and dirty
    Eigen::Vector3d where = p + direction;
    camera()->setCenter( QVector3D( where.x(), where.y(), where.z() ) );
    m_sceneCenter = QVector3D( scene_center.x(), scene_center.y(), scene_center.z() );
}

} } } // namespace snark { namespace graphics { namespace View {
