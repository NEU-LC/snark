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

#pragma once

#if Qt3D_VERSION==2

#include "../../qt3d/qt3d_v2/gl/widget.h"
#include <QMainWindow>
#include <vector>
#include <memory>
#include "reader.h"
#ifndef Q_MOC_RUN
#include <boost/property_tree/ptree.hpp>
#include "boost/optional.hpp"
#include "boost/scoped_ptr.hpp"
#endif
#include "camera_reader.h"

// todo
// - rename controller into something more meaningful; what does controller control?
// - add doxygen documentation (a line or two)
// - more color_t definition into a proper namespace (currently it is not namespaced at all)
// - what is the difference between view class and controller class? why one needs to be derived from the other? can it be just one class?
// - since all gets refactored anyway, move traits to traits.h to improve compilation time

typedef snark::graphics::qt3d::gl::color_t color_t;

namespace snark { namespace graphics { namespace qt3d {
class camera_options;
} } }

namespace snark { namespace graphics { namespace view {

class view : public qt3d::gl::widget
{
protected:
    QVector3D scene_center;
    double scene_radius;
    bool scene_radius_fixed_;
    bool scene_center_fixed_;
public:
    view(const color_t& background_color, const qt3d::camera_options& camera_options, const QVector3D& scene_center, double scene_radius,QMainWindow* parent=NULL) : 
        qt3d::gl::widget(camera_options,parent),
        scene_center(scene_center),
        scene_radius(scene_radius)
        {
            scene_radius_fixed_=false;
            scene_center_fixed_=false;
        }
    view(const qt3d::camera_options& camera_options,QMainWindow* parent=NULL) : qt3d::gl::widget(camera_options,parent) { }
//     double scene_radius() const { return scene_radius_; }
protected:
    void updateView( const QVector3D& min, const QVector3D& max )
    {
        if( !scene_radius_fixed_ ) { scene_radius = 0.5 * ( max - min ).length(); }
        if( !scene_center_fixed_ ) { scene_center = 0.5 * ( min + max ); }
//         updateZFar();
    }
    void lookAtCenter()
    {
        double r = 1.5 * scene_radius;
        QVector3D eye( scene_center.x() + r, scene_center.y() + r, scene_center.z() - r );
//         camera_.setToIdentity();
//         camera_.lookAt(eye,scene_center,QVector3D(0,0,-1));
    }
    boost::optional< Eigen::Vector3d > m_offset;

//from QGLView
//     QGLCamera * camera() const;
};
    
class controller : public view
{
    Q_OBJECT
public:
    std::vector< std::unique_ptr< snark::graphics::view::Reader > > readers;
    controller(const qt3d::camera_options& camera_options,QMainWindow* parent=NULL);
    /// @todo split into several constructors; make camera configuration a separate class
    controller( const color_t& background_color, const qt3d::camera_options& camera_options, bool exit_on_end_of_input
              , boost::optional< comma::csv::options > cameracsv //= boost::optional< comma::csv::options >()
              , boost::optional< Eigen::Vector3d > cameraposition //= boost::optional< Eigen::Vector3d >()
              , boost::optional< Eigen::Vector3d > cameraorientation //= boost::optional< Eigen::Vector3d >()
              , boost::property_tree::ptree* camera_config //= NULL // massively quick and dirty
              , const QVector3D& scene_center
              , double scene_radius
              , bool output_camera_position = false,QMainWindow* parent=NULL );
    void inhibit_stdout() { m_stdout_allowed = false; }
    void shutdown();
private slots:
    void read();

private:
//         void initializeGL( QGLPainter *painter );
//         void paintGL( QGLPainter *painter );
    void setCameraPosition( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation );
//     virtual void mouse_double_right_click_event( QMouseEvent *e );
    
    bool m_shutdown;
    bool m_lookAt;
    boost::scoped_ptr< CameraReader > m_cameraReader;
    boost::optional< Eigen::Vector3d > m_cameraposition;
    boost::optional< Eigen::Vector3d > m_cameraorientation;
    bool m_cameraFixed;
    //add camera_position_output
    bool m_stdout_allowed;
    bool m_exit_on_end_of_input;
    
public:
    void add(std::unique_ptr<snark::graphics::view::Reader>&& reader);
    
protected:
    //should be called once when opengl is setup
    void init();
    void update_shapes();
};

    
} } } // namespace snark { namespace graphics { namespace view {
    
namespace comma { namespace visiting {

template <> struct traits< QVector3D >
{
    template < typename Key, class Visitor >
    static void visit( Key, QVector3D& p, Visitor& v )
    {
        double d;
        d= p.x(); v.apply( "x", d ); p.setX( d );
        d = p.y(); v.apply( "y", d ); p.setY( d );
        d = p.z(); v.apply( "z", d ); p.setZ( d );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const QVector3D& p, Visitor& v )
    {
        v.apply( "x", p.x() );
        v.apply( "y", p.y() );
        v.apply( "z", p.z() );
    }
};

    
} } // namespace comma { namespace visiting {
   

#endif
