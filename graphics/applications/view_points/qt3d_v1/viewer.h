// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#pragma once

#ifdef WIN32
#include <winsock2.h>
//#include <windows.h>
#endif

#ifndef Q_MOC_RUN
#include <boost/property_tree/ptree.hpp>
#endif
#include <iostream>
#include <memory>
#include "../../../qt3d/qt3d_v1/view.h"
#include "../camera_reader.h"
#include "../reader.h"

namespace snark { namespace graphics { namespace view {

class Viewer : public qt3d::view
{
    Q_OBJECT
    public:
        std::vector< std::unique_ptr< Reader > > readers;

        /// @todo split into several constructors; make camera configuration a separate class
        Viewer( const QColor4ub& background_color
              , const qt3d::camera_options& camera_options
              , bool exit_on_end_of_input
              , boost::optional< comma::csv::options > cameracsv = boost::optional< comma::csv::options >()
              , boost::optional< Eigen::Vector3d > cameraposition = boost::optional< Eigen::Vector3d >()
              , boost::optional< Eigen::Vector3d > cameraorientation = boost::optional< Eigen::Vector3d >()
              , const std::string& camera_config_file_name = "" // quick and dirty
              , boost::optional< Eigen::Vector3d > scene_center = boost::optional< Eigen::Vector3d >()
              , boost::optional< double > scene_radius = boost::optional< double >()
              , bool output_camera_config = false );

        void inhibit_stdout() { m_stdout_allowed = false; }

        void shutdown();
        
        void add(std::unique_ptr<snark::graphics::view::Reader>&& reader);
        void update_view() { update(); }
        void load_camera_config(const std::string& file_name);
        void write_camera_config(std::ostream& os);

        //moved here from reader
public:
        void draw_label( QGLPainter* painter, const Eigen::Vector3d& position, const QColor4ub& color, const std::string& label );
private:
        void drawText( QGLPainter *painter, const QString& string, const QColor4ub& color );

    public slots:
        void toggle_block_mode(bool) { std::cerr << "view-points: qt4: block mode: not implemented" << std::endl; } // quick and dirty
        void toggle_label_mode(bool) { std::cerr << "view-points: qt4: label mode: not implemented" << std::endl; } // quick and dirty
        
    private slots:
        void read();        

    private:
        void initializeGL( QGLPainter *painter );
        void paintGL( QGLPainter *painter );
        void set_camera_position( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation );
        virtual void mouse_double_right_click_event( QMouseEvent *e );

        bool m_shutdown;
        bool m_lookAt;
        boost::scoped_ptr< CameraReader > m_cameraReader;
        boost::optional< Eigen::Vector3d > m_cameraposition;
        boost::optional< Eigen::Vector3d > m_cameraorientation;
        bool m_cameraFixed;
        bool m_stdout_allowed;
        bool m_exit_on_end_of_input;
        bool output_camera_config;
        void look_at_center() { lookAtCenter(); }
        void update_view(const QVector3D& min, const QVector3D& max) { updateView(min,max); }
};

} } } // namespace snark { namespace graphics { namespace view {
