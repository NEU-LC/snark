// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2020 Vsevolod Vlaskine

#include <iomanip>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <QChar>
#include <QGuiApplication>
#include <QTimer>
#include <QVector3D>
#include "../../../qt5.5/qopengl/traits.h"
#ifndef Q_MOC_RUN
#include <boost/property_tree/json_parser.hpp>
#include <comma/name_value/ptree.h>
#include <comma/visiting/apply.h>
#endif
#include "viewer.h"

namespace snark { namespace graphics { namespace view { namespace qopengl {

std::ostream& operator<<( std::ostream& os, const QVector3D& v ) { return os << v.x() << "," << v.y() << "," << v.z(); }

viewer::viewer( controller_base* handler
              , const color_t& background_color
              , const qt3d::camera_options& camera_options
              , const QVector3D& arg_scene_center
              , double arg_scene_radius
              , const snark::graphics::view::click_mode& click_mode
              , QMainWindow* parent )
    : snark::graphics::qopengl::widget( background_color, camera_options, parent )
    , scene_center( arg_scene_center )
    , handler( handler )
    , scene_radius_fixed( false )
    , scene_center_fixed( false )
    , stdout_allowed( true )
    , click_mode( click_mode )
{
    scene_radius = arg_scene_radius;
    QTimer* timer = new QTimer( this );
    connect( timer, SIGNAL( timeout() ), this, SLOT( on_timeout() ) );
    timer->start( 40 );
}

void viewer::reset_handler( controller_base* h ){ handler = h; }

void viewer::init() { if( handler != nullptr ) { handler->init(); } }

void viewer::on_timeout() { if( handler != nullptr ) { handler->tick(); } }

void viewer::paintGL()
{
    widget::paintGL();
    if( output_camera_config && stdout_allowed ) { write_camera_config( std::cout ); }
    QPainter painter( this );
    painter.setPen( Qt::gray );
    painter.setFont( QFont( "Arial", 10 ) );
    painter.drawText( rect(), Qt::AlignLeft | Qt::AlignBottom, QString::fromStdString( click_mode.double_right_click.to_info_string() ) );
}

void viewer::double_right_click(const boost::optional< QVector3D >& point)
{
    if( !stdout_allowed ) { std::cerr << "view-points: point under mouse output is disabled when --pass option is in use" << std::endl; return; }
    if( !point ) { std::cerr << "view-points: warning: no point found near the double right click" << std::endl; return; }
    Eigen::Vector3d p( point->x(), point->y(), point->z() );
    if( !m_offset ) { std::cerr << "view-points: warning: offset is not defined yet, wait until it is found first" << std::endl; return; }
    p += *m_offset;
    std::cout << std::setprecision( 16 ) << p.x() << "," << p.y() << "," << p.z() << click_mode.double_right_click.to_output_string() << std::endl;
}

void viewer::keyPressEvent( QKeyEvent *event )
{
    click_mode.double_right_click.on_key_press( event );
    update();
}

void viewer::toggle_block_mode( bool flag ) { click_mode.double_right_click.toggle( snark::graphics::view::click_mode::double_right_click_t::modes::block, flag ); }

void viewer::toggle_label_mode( bool flag ) { click_mode.double_right_click.toggle( snark::graphics::view::click_mode::double_right_click_t::modes::label, flag ); }

void viewer::update_view(const QVector3D& min, const QVector3D& max)
{
    if( !scene_radius_fixed ) { scene_radius = 0.5 * ( max - min ).length(); }
    if( !scene_center_fixed ) { scene_center = 0.5 * ( min + max ); }
//     std::cerr<<"viewer::update_view "<<min<<" "<<max<<"; "<<scene_radius<<"; "<<scene_center<<std::endl;
//     update the position of the far plane so that the full scene is displayed
    set_far_plane(4.6*scene_radius);
}

void viewer::look_at_center()
{
//     std::cerr<<"look_at_center "<<scene_center<<"; "<<scene_radius<<std::endl;
    camera.set_center(scene_center);
    camera.set_orientation(3*M_PI/4, -M_PI/4, -M_PI/4);
    camera.set_position(QVector3D(0,0,-2.6*scene_radius));
}

void viewer::set_camera_position(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation)
{
    Eigen::Vector3d p = position - *m_offset;
    double cos_yaw = std::cos( orientation.z() );
    double sin_yaw = std::sin( orientation.z() );
    double sin_pitch = std::sin( orientation.y() );
    double cos_pitch = std::cos( orientation.y() );
    Eigen::Vector3d direction( cos_pitch * cos_yaw, cos_pitch * sin_yaw, sin_pitch ); // todo: quick and dirty, forget about roll for now
    Eigen::Vector3d c = p + direction * 50; // quick and dirty
    scene_center=QVector3D(c.x(),c.y(),c.z());
    //to be tested
    camera.set_center(scene_center);    //set center
//     camera.set_orientation(orientation.x(),M_PI-orientation.y(),-M_PI/2-orientation.z());
    camera.set_orientation(orientation.x(),orientation.y(),orientation.z());
    camera.set_position(QVector3D(0,0,-p.norm()));    //camera is in 0,0,-z in world coordinate
}

void viewer::load_camera_config(const std::string& file_name)
{
    boost::property_tree::ptree camera_config;
    boost::property_tree::read_json( file_name, camera_config );
    comma::from_ptree from_ptree( camera_config, true );
    comma::visiting::apply( from_ptree ).to( camera );
}

void viewer::write_camera_config(std::ostream& os)
{
    boost::property_tree::ptree p;
    comma::to_ptree to_ptree( p );
    comma::visiting::apply( to_ptree ).to( camera );
    boost::property_tree::write_json( os, p );
}

} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {

