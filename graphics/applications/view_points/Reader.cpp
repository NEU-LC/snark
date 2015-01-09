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
//    This product includes software developed by the University of Sydney.
// 4. Neither the name of the University of Sydney nor the
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
#endif
#include <deque>
#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <snark/graphics/qt3d/rotation_matrix.h>
#include "./Reader.h"
#include "./Texture.h"

namespace snark { namespace graphics { namespace View {

Reader::Reader( QGLView& viewer, comma::csv::options& options, std::size_t size, coloured* c, unsigned int pointSize, const std::string& label, const QVector3D& offset )
    : size( size )
    , pointSize( pointSize )
    , options( options )
    , m_viewer( viewer )
    , m_num_points( 0 )
    , m_colored( c )
    , m_shutdown( false )
    , m_isStdIn( options.filename == "-" )
    , m_show( true )
    , m_istream( options.filename, options.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking )
    , updated_( false )
    , id_( 0 )
    , m_label( label )
    , m_offset( offset )
{
    std::vector< std::string > v = comma::split( options.fields, ',' ); // quick and dirty
}

void Reader::shutdown()
{
    m_shutdown = true;
    if( m_thread ) { m_thread->join(); }
    m_istream.close();
}

bool Reader::isShutdown() const { return m_shutdown; }

void Reader::show( bool s ) { m_show = s; }

bool Reader::show() const { return m_show; }

void Reader::read()
{
    for( ; !m_shutdown && read_once(); ++m_num_points );
    std::cerr << "view-points: end of " << options.filename << "; read " << m_num_points << " record(s)" << std::endl;
    m_shutdown = true;
}

bool Reader::updatePoint( const Eigen::Vector3d& offset )
{
    if( !m_point ) { return false; } // is it safe to do it without locking the mutex?
    boost::mutex::scoped_lock lock( m_mutex );
    if( !updated_ ) { return false; }
    m_translation = QVector3D( m_point->x(), m_point->y(), m_point->z() );
    m_offset = QVector3D( offset.x(), offset.y(), offset.z() );
    if( m_orientation )
    {
        const Eigen::Quaterniond& q = snark::rotation_matrix( *m_orientation ).quaternion();
        m_quaternion = QQuaternion( q.w(), q.x(), q.y(), q.z() );
    }
    updated_ = false;
    return true;
}

void Reader::draw_label( QGLPainter *painter, const QVector3D& position, const std::string& label ) { draw_label( painter, position, m_color, label ); }

void Reader::draw_label( QGLPainter *painter, const QVector3D& position, const QColor4ub& color ) { draw_label( painter, position, color, m_label ); }

void Reader::draw_label( QGLPainter *painter, const QVector3D& position ) { draw_label( painter, position, m_color, m_label ); }

void Reader::draw_label( QGLPainter *painter, const QVector3D& position, const QColor4ub& color, const std::string& label )
{
    if( label.empty() ) { return; }
    painter->modelViewMatrix().push();
    painter->modelViewMatrix().translate( position - m_offset ); // painter->modelViewMatrix().translate( position );
    QMatrix4x4 world = painter->modelViewMatrix().top();
    Eigen::Matrix3d R;
    R << world( 0, 0 ) , world( 0, 1 ), world( 0, 2 ),
         world( 1, 0 ) , world( 1, 1 ), world( 1, 2 ),
         world( 2, 0 ) , world( 2, 1 ), world( 2, 2 );
    R.transposeInPlace();
    snark::rotation_matrix rotation( R );
    Eigen::Quaterniond q = rotation.quaternion();
    painter->modelViewMatrix().rotate( QQuaternion( q.w(), q.x(), q.y(), q.z() ) );
    //painter->modelViewMatrix().translate( m_offset );
    double scale = 1.0 / double( m_viewer.height() );
    scale *= m_viewer.camera()->projectionType() == QGLCamera::Orthographic
           ? ( 0.25 * m_viewer.camera()->viewSize().width() )
           : ( 0.2 * Eigen::Vector3d( world( 0, 3 ) , world( 1, 3 ), world( 2, 3 ) ).norm() );
    painter->modelViewMatrix().scale( scale ); // TODO make size configurable ?
    drawText( painter, &label[0], color );
    painter->modelViewMatrix().pop();
}

void Reader::drawText( QGLPainter *painter, const QString& string, const QColor4ub& color )
{
    Texture texture( string, color );
    texture.draw( painter );
}

} } } // namespace snark { namespace graphics { namespace View {
