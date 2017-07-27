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

#include "controller.h"
#include "reader.h"

#include <signal.h>
#ifndef Q_MOC_RUN
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/thread_time.hpp>
#include <comma/csv/stream.h>
#include <comma/name_value/ptree.h>
#include <comma/visiting/apply.h>
#endif

#if Qt3D_VERSION==2

namespace snark { namespace graphics { namespace view {

void controller::add(std::unique_ptr<snark::graphics::view::Reader>&& reader)
{
#if Qt3D_VERSION==2
    std::shared_ptr<snark::graphics::qt3d::gl::shape> shape=reader->make_shape();
    if(shape)
    {
        viewer->shapes.push_back(shape);
    }
    std::shared_ptr<snark::graphics::qt3d::gl::label_shader> label_shader=reader->make_label_shader();
    if(label_shader)
    {
        viewer->label_shaders.push_back(label_shader);
    }
#endif
    readers.push_back(std::move(reader));
}

void controller::init()
{
    if( m_cameraReader ) { m_cameraReader->start(); }
    for(auto& i : readers) { i->start(); }
}

void controller::update_shapes()
{
#if Qt3D_VERSION==2
    viewer->begin_update();
    for(auto& i : readers)
    {
        i->update_shape();
        i->update_labels();
    }
    viewer->end_update();
#endif
}

controller::controller(QMainWindow* parent, const color_t& background_color
              , const qt3d::camera_options& camera_options
              , bool exit_on_end_of_input
              , boost::optional< comma::csv::options > camera_csv
              , boost::optional< Eigen::Vector3d > cameraposition
              , boost::optional< Eigen::Vector3d > cameraorientation
              , boost::property_tree::ptree* camera_config
              , const QVector3D& scene_center
              , double scene_radius
              , bool output_camera_position)
    : m_lookAt( false )
    , m_cameraposition( cameraposition )
    , m_cameraorientation( cameraorientation )
    , m_exit_on_end_of_input( exit_on_end_of_input )
{
#if Qt3D_VERSION==1
//     viewer=new viewer_t(background_color, camera_options, scene_center, scene_radius,parent);
    COMMA_THROW( comma::exception," not implemented ");
#elif Qt3D_VERSION==2
    viewer=new viewer_t(this,background_color, camera_options, scene_center, scene_radius,parent);
    parent->setCentralWidget(viewer);
#endif
//     if( output_camera_position ) { camera_position_output_.reset( new camera_position_output( *this ) ); }
    if( camera_csv ) { m_cameraReader.reset( new CameraReader( *camera_csv ) ); }
    if( camera_config )
    {
        comma::from_ptree from_ptree( *camera_config, true );
//         comma::visiting::apply( from_ptree ).to( *camera() );
    }
    m_cameraFixed = m_cameraposition || m_cameraReader || camera_config;
}

controller::~controller()
{
    //don't delete viewer it will be deleted by main window
    viewer->reset_handler();
//     delete viewer;
//     viewer=NULL;
    shutdown(false);
}
void controller::inhibit_stdout() { viewer->stdout_allowed = false; }
void controller::shutdown(bool kill)
{
    m_shutdown = true;
    if(kill)
    {
    #ifdef WIN32
        ::raise( SIGTERM ); // according to windows docs, SIGINT does not work on windows
    #else // #ifdef WIN32
        ::raise( SIGINT ); // quick and dirty... or maybe not so dirty: to interrupt blocking read() in reader threads, if closed from qt window
    #endif // #ifdef WIN32
    }
    if( m_cameraReader ) { m_cameraReader->shutdown(); }
    for(auto& i : readers) { i->shutdown(); }
}

void controller::read()
{
//     if(viewer==NULL) return;
    for( unsigned int i = 0; !viewer->m_offset && i < readers.size(); ++i )
    {
        if( readers[i]->empty() ) { continue; }
        Eigen::Vector3d p = readers[i]->somePoint();
        viewer->m_offset = std::fabs( p.x() ) > 1000 || std::fabs( p.y() ) > 1000 || std::fabs( p.z() ) > 1000 ? p : Eigen::Vector3d( 0, 0, 0 );
        std::cerr << "view-points: reader no. " << i << " scene offset (" << viewer->m_offset->transpose() << "); scene radius: " << viewer->scene_radius << std::endl;
    }
    if( !viewer->m_offset ) { return; }
    bool need_update = false;
    for( unsigned int i = 0; i < readers.size(); ++i )
    {
        if( readers[i]->update( *viewer->m_offset ) > 0 ) { need_update = true; };
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
        viewer->set_camera_position( *m_cameraposition, *m_cameraorientation );
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
            viewer->set_camera_position( position, orientation );
        }
    }
    else if( readers[0]->m_extents && readers[0]->m_num_points > 0 && ( m_shutdown || ready_to_look || readers[0]->m_num_points >= readers[0]->size / 10 ) )
    {
        QVector3D min( readers[0]->m_extents->min().x(), readers[0]->m_extents->min().y(), readers[0]->m_extents->min().z() );
        QVector3D max( readers[0]->m_extents->max().x(), readers[0]->m_extents->max().y(), readers[0]->m_extents->max().z() );
        viewer->update_view( min, max );
        if( !m_cameraFixed && !m_lookAt )
        {
            m_lookAt = true;
            viewer->look_at_center();
        }
    }
    if( need_update )
    {
        update_shapes();
        viewer->update();
    }
    if( m_shutdown && m_exit_on_end_of_input ) { shutdown(); }
}

} } } // namespace snark { namespace graphics { namespace view {

#endif
