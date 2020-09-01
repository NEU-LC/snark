// Copyright (c) 2017 The University of Sydney

#pragma once

#include <comma/base/types.h>
#include "../../../qt5.5/qopengl/widget.h"
#include "../types.h"
#include <QKeyEvent>
#include <QMainWindow>

namespace snark { namespace graphics { namespace qt3d {
class camera_options;
} } }
typedef snark::graphics::qopengl::color_t color_t;

namespace snark { namespace graphics { namespace view { namespace qopengl {

/**
 * redner and camera functions
 * qt3d v2 specific rednering, most functions are implemented in widget
 * this class implements interface used by controller
 */
class viewer : public snark::graphics::qopengl::widget
{
    Q_OBJECT
public:
    QVector3D scene_center;
    boost::optional< Eigen::Vector3d > m_offset;
    controller_base* handler;
    bool scene_radius_fixed;
    bool scene_center_fixed;
    bool stdout_allowed;
    bool output_camera_config;
    bool block_mode;
    comma::uint32 block;

    public:
    viewer(controller_base* handler, const color_t& background_color, const qt3d::camera_options& camera_options,
           const QVector3D& scene_center, double scene_radius, QMainWindow* parent = nullptr);
    void reset_handler(controller_base* h = nullptr);

protected:
    void init() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void keyPressEvent( QKeyEvent *event ) Q_DECL_OVERRIDE;
    void double_right_click(const boost::optional<QVector3D>& point) override;

private slots:
    void on_timeout();

public:
    void look_at_center();
    void load_camera_config(const std::string& file_name);
    void set_camera_position(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation);
    void update_view(const QVector3D& min, const QVector3D& max);
    void write_camera_config(std::ostream& os);

public slots:
    void toggle_block_mode(bool);

//     double scene_radius() const { return scene_radius_; }
};


} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {

