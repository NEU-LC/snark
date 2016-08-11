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


#include <cmath>
#include <vector>
#ifndef Q_MOC_RUN
#include <boost/array.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/math/compare.h>
#endif
#include "Viewer.h"

namespace snark { namespace graphics { namespace view {

Viewer::Viewer( const std::vector< comma::csv::options >& options
              , const qt3d::camera_options& camera_options
              , const comma::csv::options& csv_out
              , bool labelDuplicated
              , const QColor4ub& background_color
              , bool verbose )
    : qt3d::view( background_color, camera_options )
    , navigate( *this )
    , pickId( *this )
    , selectPartition( *this )
    , selectId( *this )
    , selectClip( *this )
    , fill( *this )
    , m_currentTool( &navigate )
    , m_options( options )
    , output_with_id( csv_out.has_field( "id" ) )
    , m_output_stream( std::cout, csv_out )
    , m_labelDuplicated( labelDuplicated )
    , verbose_( verbose )
{
    // quick and dirty, since otherwise dataset construction throws on a separate thread on non-existing file and then segfaults on exit
    for( std::size_t i = 0; i < m_options.size(); ++i )
    {
        if( !boost::filesystem::exists( m_options[i].filename ) ) { COMMA_THROW( comma::exception, "file " << m_options[i].filename << " does not exist" ); }
    }
}

void Viewer::saveStateToFile() {}

void Viewer::show( std::size_t i, bool visible )
{
    m_datasets[i]->visible( visible );
    update();
}

void Viewer::setWritable( std::size_t i, bool writable )
{
    m_datasets[i]->writable( writable );
}

void Viewer::save()
{
    for( std::size_t i = 0; i < m_datasets.size(); ++i )
    {
        if( m_datasets[i]->writable() ) { m_datasets[i]->save(); }
    }
}

const std::vector< boost::shared_ptr< Dataset > >& Viewer::datasets() const { return m_datasets; }

Dataset& Viewer::dataset( std::size_t index ) { return *m_datasets[index]; }

const Dataset& Viewer::dataset( std::size_t index ) const { return *m_datasets[index]; }

void Viewer::reload()
{
    for( std::size_t i = 0; i < m_datasets.size(); ++i )
    {
        std::string filename = m_datasets[i]->filename();
        comma::csv::options options = m_datasets[i]->options();
        bool writable = m_datasets[i]->writable();
        bool visible = m_datasets[i]->visible();
        m_datasets[i].reset();
        m_datasets[i].reset( new Dataset( filename, options, *m_offset, m_labelDuplicated ) );
        if( !m_datasets[i]->valid() ) { std::cerr << "label-points: failed to reload datasets" << std::endl; exit( -1 ); }
        m_datasets[i]->init();
        m_datasets[i]->writable( writable );
        m_datasets[i]->visible( visible );
    }
    setCamera();
    update();
}

void Viewer::setCamera()
{
    assert( !m_datasets.empty() );
    snark::math::closed_interval< double, 3 > extents = m_datasets[0]->extents();
    for( std::size_t i = 0; i < m_datasets.size(); ++i ) { extents = extents.hull( m_datasets[i]->extents() ); }
    extents = extents.hull( extents.min() - Eigen::Vector3d( 10, 10, 10 ) ); // expand a bit
    extents = extents.hull( extents.max() + Eigen::Vector3d( 10, 10, 10 ) ); // expand a bit
    Eigen::Vector3d minCorner = extents.min() - m_datasets[0]->offset();
    Eigen::Vector3d maxCorner = extents.max() - m_datasets[0]->offset();
    QVector3D min( minCorner.x(), minCorner.y(), minCorner.z() );
    QVector3D max( maxCorner.x(), maxCorner.y(), maxCorner.z() );
    updateView( min, max );
    lookAtCenter();
    updateZFar();
}

void Viewer::initializeGL( QGLPainter *painter )
{
    ::glDisable( GL_LIGHTING );
//     setBackgroundColor( QColor( m_background_color.red(), m_background_color.green(), m_background_color.blue() ) );
    m_datasets.push_back( boost::shared_ptr< Dataset >( new Dataset( m_options[0].filename, m_options[0], m_labelDuplicated ) ) );
    if( !m_datasets[0]->valid() ) { std::cerr << "label-points: failed to load dataset " << m_options[0].filename << std::endl; exit( -1 ); }
    m_datasets[0]->init();
    m_datasets[0]->writable( true );
    m_datasets[0]->visible( true );
    m_offset = m_datasets[0]->offset();
    for( std::size_t i = 1; i < m_options.size(); ++i )
    {
        m_datasets.push_back( boost::shared_ptr< Dataset >( new Dataset( m_options[i].filename, m_options[i], *m_offset, m_labelDuplicated ) ) );
        if( !m_datasets.back()->valid() ) { std::cerr << "label-points: failed to load dataset " << m_options[i].filename << std::endl; exit( -1 ); }
        m_datasets.back()->init();
        m_datasets.back()->writable( false );
        m_datasets.back()->visible( true );
    }
    setCamera();
}

void Viewer::paintGL( QGLPainter *painter )
{
    updateZFar();
    static bool emitted = false;
    if( !emitted ) { emitted = true; emit initialized(); } // real quick and dirty: somehow emit fails in init()
    ::glPointSize( 1 );
    int selectionPointSize = 1;
    for( std::size_t i = 0; i < m_datasets.size(); ++i )
    {
        m_datasets[i]->draw( painter );
        if( m_datasets[i]->visible() ) { selectionPointSize = 3; }
    }
    ::glEnable( GL_POINT_SMOOTH );
    ::glPointSize( selectionPointSize );
    for( std::size_t i = 0; i < m_datasets.size(); ++i ) { m_datasets[i]->selection().draw( painter ); }
    ::glDisable( GL_POINT_SMOOTH );
    m_currentTool->draw( painter );
    draw_coordinates( painter );
}

void Viewer::mousePressEvent( QMouseEvent* e )
{
    m_currentTool->onMousePress( e );
//     GL::View::mousePressEvent( e );
}

void Viewer::mouseReleaseEvent( QMouseEvent* e )
{
    m_currentTool->onMouseRelease( e );
//     GL::View::mouseReleaseEvent( e );
}

void Viewer::mouseMoveEvent( QMouseEvent *e )
{
    m_currentTool->onMouseMove( e );
//     GL::View::mouseMoveEvent( e );
}

boost::optional< point_and_id > Viewer::pointSelection( const QPoint& point, bool writableOnly )
{
    boost::optional< point_and_id > result;
    boost::optional< QVector3D > point3d = getPoint( point );
    if( point3d )
    {
        Eigen::Vector3d p(  point3d->x(), point3d->y(), point3d->z() );
        if( m_offset ) { p += *m_offset; }
        if( !output_with_id ) { m_output_stream.write( PointWithId( p ) ); }
        std::cerr << " clicked point " << std::setprecision( 12 ) << p.transpose() << std::endl;
        snark::math::closed_interval< double, 3 > e( p - Eigen::Vector3d::Ones(), p + Eigen::Vector3d::Ones() );
        double minDistanceSquare = std::numeric_limits< double >::max();
        for( std::size_t i = 0; i < m_datasets.size(); ++i )
        {
            if( !m_datasets[i]->visible() || ( writableOnly && !m_datasets[i]->writable() ) ) { continue; }
            Dataset::Points m = m_datasets[i]->points().find( e.min(), e.max() );
            if( verbose_ ) { std::cerr << " found " << m.size() << " point(s) in the neighbourhood" << std::endl; }
            for( Dataset::Points::ConstEnumerator en = m.begin(); !en.end(); ++en )
            {
                double squared_norm = ( en.key() - p ).squaredNorm();
                if( squared_norm < minDistanceSquare )
                {
                    minDistanceSquare = squared_norm;
                    result = std::make_pair( en.key(), en.value().id );
                }
            }
            if( minDistanceSquare <= 0.01 )
            {
                if( output_with_id ) { m_output_stream.write( PointWithId( *result ) ); }
                if( verbose_ ) { std::cerr << " found point: " << std::setprecision( 12 ) << result->first.transpose() << "; id: " << result->second << std::endl; }
                return result;
            }
        }
    }
    return boost::optional< point_and_id >();
}

void Viewer::handleId( comma::uint32 id ) { m_id = id; }

} } } // namespace snark { namespace graphics { namespace view {
