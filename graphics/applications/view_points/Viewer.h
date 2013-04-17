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

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_VIEWER_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_VIEWER_H_

#ifdef WIN32
#include <winsock2.h>
//#include <windows.h>
#endif

#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <snark/graphics/qt3d/view.h>
#include "./CameraReader.h"
#include "./Reader.h"

namespace snark { namespace graphics { namespace View {

class Viewer : public qt3d::view
{
    Q_OBJECT
    public:
        std::vector< boost::shared_ptr< Reader > > readers;

        Viewer( const QColor4ub& background_color
              , double fov
              , bool z_up
              , bool orthographic = false
              , boost::optional< comma::csv::options > cameracsv = boost::optional< comma::csv::options >()
              , boost::optional< Eigen::Vector3d > cameraposition = boost::optional< Eigen::Vector3d >()
              , boost::optional< Eigen::Vector3d > cameraorientation = boost::optional< Eigen::Vector3d >()
              , boost::optional< Eigen::Vector3d > scene_center = boost::optional< Eigen::Vector3d >()
              , boost::optional< double > scene_radius = boost::optional< double >() );

        void shutdown();

    private slots:
        void read();

    private:
        void initializeGL( QGLPainter *painter );
        void paintGL( QGLPainter *painter );
        void setCameraPosition( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation );

        bool m_shutdown;
        bool m_lookAt;
        boost::scoped_ptr< CameraReader > m_cameraReader;
        boost::optional< Eigen::Vector3d > m_cameraposition;
        boost::optional< Eigen::Vector3d > m_cameraorientation;
        bool m_cameraFixed;
};

} } } // namespace snark { namespace graphics { namespace View {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_VIEWER_H_*/
