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

#pragma once

#if Qt3D_VERSION==1
#include "qt3d_v1/viewer.h"

#elif Qt3D_VERSION==2
#include "qopengl/viewer.h"

#endif

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
#include "types.h"

namespace snark { namespace graphics { namespace view {
    
template<typename T>
struct controller_traits
{
    static viewer_t* get_widget(std::shared_ptr<T>& t) { return NULL; }
};
#if Qt3D_VERSION==1
//mock controller for now, TODO refactor Viewer and controller
typedef Viewer controller;

template<>
struct controller_traits<controller>
{
    static viewer_t* get_widget(std::shared_ptr<controller>& t) { return t.get(); }
};

#elif Qt3D_VERSION==2
/**
 * manages readers and camera
 * contains a viewer which performs the rendering and is Qt3d version dependant
 */
class controller : public  controller_base
{
public:
    std::shared_ptr<viewer_t> viewer;
    std::vector< std::unique_ptr< snark::graphics::view::Reader > > readers;
    
    /// @todo split into several constructors; make camera configuration a separate class
    controller(const color_t& background_color, const qt3d::camera_options& camera_options, bool exit_on_end_of_input
              , boost::optional< comma::csv::options > cameracsv //= boost::optional< comma::csv::options >()
              , boost::optional< Eigen::Vector3d > cameraposition //= boost::optional< Eigen::Vector3d >()
              , boost::optional< Eigen::Vector3d > cameraorientation //= boost::optional< Eigen::Vector3d >()
              , const std::string& camera_config_file_name //="" pass empty string, for not loading any config
              , const QVector3D& scene_center
              , double scene_radius
              , bool output_camera_config = false);
    ~controller();
    void inhibit_stdout();
    void shutdown(bool kill=true);
    void tick() { read(); }

    void read();
    void update_view();
    void load_camera_config(const std::string& file_name);
    void write_camera_config(std::ostream& os);

private:
    bool m_shutdown;
    bool m_lookAt;
    boost::scoped_ptr< CameraReader > m_cameraReader;
    boost::optional< Eigen::Vector3d > m_cameraposition;
    boost::optional< Eigen::Vector3d > m_cameraorientation;
    bool m_cameraFixed;
    //add camera_position_output
    bool m_exit_on_end_of_input;
    
public:
    void add(std::unique_ptr<snark::graphics::view::Reader>&& reader);
    void init();
};
template<>
struct controller_traits<controller>
{
    static viewer_t* get_widget(std::shared_ptr<controller>& t) { return t->viewer.get(); }
};
#endif
    
} } } // namespace snark { namespace graphics { namespace view {
    
