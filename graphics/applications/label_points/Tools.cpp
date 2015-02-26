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


#include <algorithm>
#include <boost/array.hpp>
#include "Viewer.h"
#include "Tools.h"
#include <Qt3D/qglcube.h>

namespace snark { namespace graphics { namespace View {  namespace Tools {

namespace impl { // quick and dirty

static boost::array< unsigned int, 256 > colorInit()
{
    boost::array< unsigned int, 256 > a;
    for( std::size_t i = 0; i < 256; ++i ) { a[i] = i; }
    std::random_shuffle( a.begin(), a.end() );
    return a;
}
static unsigned char colorIndex = 0;
static boost::array< unsigned int, 256 > colorIndices = colorInit();
static void colorShake() { std::random_shuffle( colorIndices.begin(), colorIndices.end() ); ++colorIndex; }

} // namespace impl {

QColor4ub colorFromId( comma::uint32 id ) // quick and dirty, arbitrary
{
    static const float b = 243;
    unsigned char i = ( id & 0xff ) + ( ( id & 0xff00 ) >> 16 );
    const float h = b * float( impl::colorIndices[ i ] );
    const float a = 0.3 * float( 255 - impl::colorIndices[ i ] );
    switch( ( i * 13 + impl::colorIndex ) % 6 )
    {
        case 0 : return QColor4ub( b, h, a );
        case 1 : return QColor4ub( a, b, h );
        case 2 : return QColor4ub( h, a, b );
        case 3 : return QColor4ub( b, a, h );
        case 4 : return QColor4ub( h, b, a );
        default: return QColor4ub( a, h, b );
    }
}

Tool::Tool( Viewer& viewer, QCursor* cursor ) : m_viewer( viewer ), m_cursor( cursor ) {}

void Tool::onMousePress( QMouseEvent* e ) { m_viewer.qt3d::view::mousePressEvent( e );}
void Tool::onMouseRelease( QMouseEvent* e ) { m_viewer.qt3d::view::mouseReleaseEvent( e ); }
void Tool::onMouseMove( QMouseEvent* e ) { m_viewer.qt3d::view::mouseMoveEvent( e ); }
void Tool::draw( QGLPainter* painter ) {}

void Tool::toggle( bool checked )
{
    if( checked )
    {
        init();
        m_viewer.setCursor( *m_cursor );
        m_viewer.m_currentTool = this;
    }
    else
    {
        reset();
    }
}

Navigate::Navigate( Viewer& viewer ) : Tool( viewer, new QCursor ) { m_cursor->setShape( Qt::ArrowCursor ); }

PickId::PickId( Viewer& viewer ) : Tool( viewer, new QCursor( Icons::pipette().pixmap( 32, 32 ) ) ) {}

void PickId::onMousePress( QMouseEvent* e )
{
    if( e->button() == Qt::LeftButton )
    {
        boost::optional< point_and_id > picked = m_viewer.pointSelection( e->pos() );
        if( picked )
        {
            m_viewer.m_output_stream.write( PointWithId( *picked ) );
            m_viewer.m_id = picked->second;
            emit valueChanged( picked->second );
        }
    }
}

void PickId::shakeColors()
{
    impl::colorShake();
    for( std::size_t i = 0; i < m_viewer.m_datasets.size(); ++i )
    {
        m_viewer.m_datasets[i]->init();
        m_viewer.m_datasets[i]->selection().init();
    }
    if( m_viewer.m_id ) { emit valueChanged( *m_viewer.m_id ); }
    m_viewer.update();
}

SelectPartition::SelectPartition( Viewer& viewer ) : Tool( viewer, new QCursor ) { m_cursor->setShape( Qt::ArrowCursor ); }

void SelectPartition::onMousePress( QMouseEvent* e )
{
    if( e->button() != Qt::LeftButton ) { return; }
    bool append = e->modifiers() == Qt::ControlModifier;
    bool erase = e->modifiers() == Qt::ShiftModifier;
    if( append && erase ) { erase = false; } // append wins
    if( !append && !erase )
    {
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i ) { m_viewer.dataset( i ).selection().clear(); }
    }
    boost::optional< point_and_id > picked = m_viewer.pointSelection( e->pos() );
    if( picked )
    {
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i )
        {
            if( !erase && !m_viewer.dataset( i ).visible() ) { continue; }
            Dataset::Partitions::const_iterator it = m_viewer.dataset( i ).partitions().find( picked->second );
            if( it == m_viewer.dataset( i ).partitions().end() ) { continue; }
            //if( it->second.find( picked->first ) == NULL ) { continue; }
            if( it->second.find( picked->first ).empty() ) { continue; }
            if( erase ) { m_viewer.dataset( i ).selection().erase( it->second ); }
            else { m_viewer.dataset( i ).selection().insert( it->second ); }
            std::cerr << "label-points: partition id " << picked->second << " with " << it->second.size() << " point(s) in " << m_viewer.dataset( i ).filename() << ( erase ? " removed from selection" : append ? " added to selection" : " selected" ) << std::endl;
            if( m_viewer.verbose() )
            {
                for( Dataset::Points::ConstEnumerator en = it->second.begin(); !en.end(); ++en )
                {
                    std::cerr << std::setprecision( 12 ) << en.key().x() << "," << en.key().y() << "," << en.key().z() << "," << en.value().id << std::endl;
                }
            }
            break;
        }
    }
    m_viewer.update();
}

void SelectPartition::onMouseMove( QMouseEvent* ) {}

void SelectPartition::onMouseRelease( QMouseEvent* ) {}

SelectId::SelectId( Viewer& viewer ) : Tool( viewer, new QCursor ) { m_cursor->setShape( Qt::ArrowCursor ); }

void SelectId::onMousePress( QMouseEvent* e )
{
    if( e->button() != Qt::LeftButton ) { return; }
    bool append = e->modifiers() == Qt::ControlModifier;
    bool erase = e->modifiers() == Qt::ShiftModifier;
    if( append && erase ) { erase = false; } // append wins
    if( !append && !erase )
    {
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i ) { m_viewer.dataset( i ).selection().clear(); }
    }
    boost::optional< point_and_id > picked = m_viewer.pointSelection( e->pos() );
    if( picked )
    {
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i )
        {
            if( !erase && !m_viewer.dataset( i ).visible() ) { continue; }
            Dataset::Partitions::const_iterator it = m_viewer.dataset( i ).partitions().find( picked->second );
            if( it == m_viewer.dataset( i ).partitions().end() ) { continue; }
            if( erase ) { m_viewer.dataset( i ).selection().erase( it->second ); }
            else { m_viewer.dataset( i ).selection().insert( it->second ); }
            std::cerr << "label-points: partition id " << picked->second << " with " << it->second.size() << " point(s) in " << m_viewer.dataset( i ).filename() << ( erase ? " removed from selection" : append ? " added to selection" : " selected" ) << std::endl;
            if( m_viewer.verbose() )
            {
                for( Dataset::Points::ConstEnumerator en = it->second.begin(); !en.end(); ++en )
                {
                    std::cerr << std::setprecision( 12 ) << en.key().x() << "," << en.key().y() << "," << en.key().z() << "," << en.value().id << std::endl;
                }
            }
        }
    }
    m_viewer.update();
}

void SelectId::onMouseMove( QMouseEvent* ) {}

void SelectId::onMouseRelease( QMouseEvent* ) {}

Fill::Fill( Viewer& viewer ) : Tool( viewer, new QCursor( Icons::bucket().pixmap( 32, 32 ) ) ) {}

void Fill::onMousePress( QMouseEvent* e )
{
    if( !m_viewer.m_id ) { return; }
    std::cerr << " id " << *m_viewer.m_id << std::endl;
    bool selectionEmpty = true;
    for( std::size_t i = 0; i < m_viewer.datasets().size() && selectionEmpty; ++i ) { selectionEmpty = m_viewer.dataset( i ).selection().points().empty(); }
    if( selectionEmpty )
    {
        boost::optional< point_and_id > picked = m_viewer.pointSelection( e->pos(), true );
        if( !picked ) { return; }
        std::cerr << " picked " << picked->first << std::endl;
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i )
        {
            if( m_viewer.dataset( i ).points().find( picked->first ).empty() ) { continue; }
            m_viewer.dataset( i ).label( picked->first, *m_viewer.m_id );
            std::cerr << "label-points: (" << picked->first << ") labeled with id " << *m_viewer.m_id << " in " << m_viewer.dataset( i ).filename() << std::endl;
        }
    }
    else
    {
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i )
        {
            if( m_viewer.dataset( i ).selection().points().empty() ) { continue; }
            for( std::size_t j = 0; j < m_viewer.datasets().size(); ++j )
            {
                if( !m_viewer.dataset( j ).writable() || !m_viewer.dataset( j ).visible() ) { continue; }
                m_viewer.dataset( j ).label( m_viewer.dataset( i ).selection().points(), *m_viewer.m_id );
                std::cerr << "label-points: labeled selection with id " << *m_viewer.m_id << " in " << m_viewer.dataset( j ).filename() << std::endl;
            }
            m_viewer.dataset( i ).selection().clear();
        }
    }
    m_viewer.update();
}

SelectClip::SelectClip( Viewer& viewer ) : Tool( viewer, new QCursor )
{
    m_cursor->setShape( Qt::ArrowCursor );
}

void SelectClip::onMousePress( QMouseEvent* e )
{
    if( e->button() != Qt::LeftButton ) { return; }
    boost::optional< point_and_id > picked = m_viewer.pointSelection( e->pos() );
    if( !picked ) { return; }
    m_rectangle = QRect( e->pos(), e->pos() );
    m_center = picked->first;
    m_radius = Eigen::Vector3d();
}

void SelectClip::onMouseMove( QMouseEvent* e )
{
    if( !m_rectangle ) { return; }
    m_rectangle->setBottomRight( e->pos() );
    double factor = ( m_viewer.camera()->eye() - m_viewer.camera()->center() ).length() / 1000;
    double h = factor * m_rectangle->height();
    double w = factor * m_rectangle->width();
    m_radius = Eigen::Vector3d( w, h, ( h + w ) / 2 );
    m_viewer.update();

}

void SelectClip::onMouseRelease( QMouseEvent* e )
{
    if( e->button() != Qt::LeftButton || !m_rectangle ) { return; }
    bool append = e->modifiers() == Qt::ControlModifier;
    bool erase = e->modifiers() == Qt::ShiftModifier;
    if( !append && !erase )
    {
        for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i ) { m_viewer.dataset( i ).selection().clear(); }
    }
    snark::math::closed_interval< double, 3 > extents( m_center - m_radius );
    extents = extents.hull( m_center + m_radius );
    for( std::size_t i = 0; i < m_viewer.datasets().size(); ++i )
    {
        if( !erase && !m_viewer.dataset( i ).visible() ) { continue; }
        Dataset::Points m = m_viewer.dataset( i ).points().find( extents.min(), extents.max() );
        if( erase ) { m_viewer.dataset( i ).selection().erase( m ); }
        else { m_viewer.dataset( i ).selection().insert( m ); }
        std::cerr << "label-points: " << m.size() << " point(s) from " << m_viewer.dataset( i ).filename() << ( erase ? " removed from selection" : append ? " added to selection" : " selected" ) << std::endl;
        if( m_viewer.verbose() )
        {
            for( Dataset::Points::ConstEnumerator en = m.begin(); !en.end(); ++en )
            {
                std::cerr << en.key().x() << "," << en.key().y() << "," << en.key().z() << "," << en.value().id << std::endl;
            }
        }
    }
    m_rectangle = boost::optional< QRect >();
    m_viewer.update();
}

void SelectClip::draw( QGLPainter* painter )
{
    if( !m_rectangle ) { return; }
    Eigen::Vector3d center = m_center;
    if( m_viewer.m_offset )
    {
        center -= *m_viewer.m_offset;
    }
    snark::math::closed_interval< double, 3 > extents( center - m_radius);
    extents = extents.hull( center + m_radius );

    QVector3D a( extents.min().x(), extents.min().y(), extents.min().z() );
    QVector3D b( extents.min().x(), extents.min().y(), extents.max().z() );
    QVector3D c( extents.min().x(), extents.max().y(), extents.max().z() );
    QVector3D d( extents.min().x(), extents.max().y(), extents.min().z() );
    QVector3D e( extents.max().x(), extents.min().y(), extents.min().z() );
    QVector3D f( extents.max().x(), extents.min().y(), extents.max().z() );
    QVector3D g( extents.max().x(), extents.max().y(), extents.max().z() );
    QVector3D h( extents.max().x(), extents.max().y(), extents.min().z() );

    QVector3DArray vertices;
    vertices.append( a );
    vertices.append( b );
    vertices.append( c );
    vertices.append( d );
    vertices.append( e );
    vertices.append( f );
    vertices.append( g );
    vertices.append( h );
    painter->setStandardEffect( QGL::FlatColor );
    painter->setVertexAttribute(QGL::Position, vertices );
    painter->draw( QGL::LineLoop, 4 );
    painter->draw( QGL::LineLoop, 4, 4 );
    boost::array< unsigned short, 8  > indices = { { 0, 4, 1, 5, 2, 6, 3, 7 } };
    painter->draw( QGL::Lines, &indices[0], 8 );
}

} } } } // namespace snark { namespace graphics { namespace View { namespace Tools {
