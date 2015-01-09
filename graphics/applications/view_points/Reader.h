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


/// @author Vsevolod Vlaskine, Cedric Wohlleber

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_READER_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_READER_H_

#include <deque>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <comma/base/types.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/file_descriptor.h>
#include <comma/io/stream.h>
#include <comma/sync/synchronized.h>
#include <snark/math/interval.h>
#include "./Coloured.h"
#include "./PointWithId.h"
#include <snark/graphics/qt3d/vertex_buffer.h>
#include <Qt3D/qglview.h>

class QGLAbstractScene;

namespace snark { namespace graphics { namespace View {

class Viewer;

class Reader
{
    public:
        const std::size_t size;
        const unsigned int pointSize;
        const comma::csv::options options;

        Reader( QGLView& viewer, comma::csv::options& options, std::size_t size, coloured* c, unsigned int pointSize, const std::string& label, const QVector3D& offset = QVector3D( 0, 0, 0 ) );

        virtual ~Reader() {}

        virtual void start() = 0;
        virtual std::size_t update( const Eigen::Vector3d& offset ) = 0;
        virtual const Eigen::Vector3d& somePoint() const = 0;
        virtual bool read_once() = 0;
        virtual void render( QGLPainter *painter ) = 0;
        virtual bool empty() const = 0;

        void show( bool s );
        bool show() const;
        bool isShutdown() const;
        bool isStdIn() const { return m_isStdIn; }
        void shutdown();
        void read();

    protected:
        bool updatePoint( const Eigen::Vector3d& offset );
        void draw_label( QGLPainter* painter, const QVector3D& position, const std::string& label );
        void draw_label( QGLPainter* painter, const QVector3D& position, const QColor4ub& color, const std::string& label );
        void draw_label( QGLPainter* painter, const QVector3D& position );
        void draw_label( QGLPainter* painter, const QVector3D& position, const QColor4ub& color );

        friend class Viewer;
        QGLView& m_viewer;
        boost::optional< snark::math::closed_interval< float, 3 > > m_extents;
        unsigned int m_num_points;
        boost::scoped_ptr< coloured > m_colored;
        bool m_shutdown;
        bool m_isStdIn;
        bool m_show;
        comma::io::istream m_istream;
        boost::scoped_ptr< boost::thread > m_thread;
        mutable boost::mutex m_mutex;
        boost::optional< Eigen::Vector3d > m_point;
        bool updated_; // quick and dirty, terrible
        boost::optional< Eigen::Vector3d > m_orientation;
        comma::uint32 id_; // todo: quick and dirty; replace m_point, m_orientation, etc with PointWithId point_;
        QColor4ub m_color;
        QVector3D m_translation;
        QQuaternion m_quaternion;
        std::string m_label;
        QVector3D m_offset;

    private:
        void drawText( QGLPainter *painter, const QString& string, const QColor4ub& color );
};

} } } // namespace snark { namespace graphics { namespace View {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_READER_H_*/
