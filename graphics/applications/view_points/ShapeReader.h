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

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_SHAPE_READER_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_SHAPE_READER_H_

#ifdef WIN32
#include <winsock2.h>
//#include <windows.h>
#endif

#include "./Reader.h"
#include "./ShapeWithId.h"

namespace snark { namespace graphics { namespace View {

template< typename S, typename How = how_t::points >
class ShapeReader : public Reader
{
    public:
        ShapeReader( QGLView& viewer, comma::csv::options& options, std::size_t size, coloured* c, unsigned int pointSize, const std::string& label, const S& sample = S() );

        void start();
        std::size_t update( const Eigen::Vector3d& offset );
        const Eigen::Vector3d& somePoint() const;
        bool readOnce();
        void render( QGLPainter *painter = NULL );
        bool empty() const;

    private:
        typedef std::deque< ShapeWithId< S > > DequeType;
        DequeType m_deque;
        mutable boost::mutex m_mutex;
        boost::scoped_ptr< comma::csv::input_stream< ShapeWithId< S > > > m_stream;
        qt3d::vertex_buffer m_buffer;
        std::vector< std::pair< QVector3D, std::string > > m_labels;
        unsigned int m_labelIndex;
        unsigned int m_labelSize;
        ShapeWithId< S > sample_;
};


template< typename S, typename How >
ShapeReader< S, How >::ShapeReader( QGLView& viewer, comma::csv::options& options, std::size_t size, coloured* c, unsigned int pointSize, const std::string& label, const S& sample  ):
    Reader( viewer, options, size, c, pointSize, label ),
    m_buffer( size * Shapetraits< S, How >::size ),
    m_labels( size ),
    m_labelIndex( 0 ),
    m_labelSize( 0 ),
    sample_( sample )
{
}

template< typename S, typename How >
inline void ShapeReader< S, How >::start()
{
    m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) );
}

template< typename S, typename How >
inline std::size_t ShapeReader< S, How >::update( const Eigen::Vector3d& offset )
{
    boost::mutex::scoped_lock lock( m_mutex );
    for( typename DequeType::iterator it = m_deque.begin(); it != m_deque.end(); ++it )
    {
        Shapetraits< S, How >::update( it->shape, offset, it->color, it->block, m_buffer, m_extents );
    }
    std::size_t s = m_deque.size();
    m_deque.clear();
    updated_ = true;
    updatePoint( offset );
    return s;
}

template< typename S, typename How >
inline bool ShapeReader< S, How >::empty() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return m_deque.empty();
}

template< typename S, typename How >
inline const Eigen::Vector3d& ShapeReader< S, How >::somePoint() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return Shapetraits< S, How >::somePoint( m_deque.front().shape );
}

template< typename S, typename How >
inline void ShapeReader< S, How >::render( QGLPainter* painter )
{
    painter->setStandardEffect(QGL::FlatPerVertexColor);
    painter->clearAttributes();
    painter->setVertexAttribute(QGL::Position, m_buffer.points() );
    painter->setVertexAttribute(QGL::Color, m_buffer.color() );

    Shapetraits< S, How >::draw( painter, m_buffer.size(), m_buffer.index() );
    for( unsigned int i = 0; i < m_labelSize; i++ ) { drawLabel( painter, m_labels[ i ].first, m_labels[ i ].second ); }
    if( !m_label.empty() ) { drawLabel( painter, m_translation ); }
}

template< typename S, typename How >
inline bool ShapeReader< S, How >::readOnce()
{
    try
    {
        if( !m_stream ) // quick and dirty: handle named pipes
        {
            if( !m_istream() )
            {
#ifndef WIN32
                // HACK poll on blocking pipe
                ::usleep( 1000 );
#endif
                return true;
            }
            m_stream.reset( new comma::csv::input_stream< ShapeWithId< S > >( *m_istream(), options, sample_ ) );
        }
        const ShapeWithId< S >* p = m_stream->read();
        if( p == NULL )
        {
            m_shutdown = true;
            return false;
        }
        ShapeWithId< S > v = *p;
        Eigen::Vector3d center = Shapetraits< S, How >::center( v.shape );
        if( !v.label.empty() )
        {
            m_labels[ m_labelIndex ] = std::make_pair( QVector3D( center.x(), center.y(), center.z() ) , v.label );
            ++m_labelIndex;
            if( m_labelSize < m_labels.size() ) { ++m_labelSize; }
            if( m_labelIndex > m_labels.size() ) { m_labelIndex = 0; }
        }
        v.color = m_colored->color( center, p->id, p->scalar, p->color );
        boost::mutex::scoped_lock lock( m_mutex );
        m_deque.push_back( v );
        m_point = Shapetraits< S, How >::somePoint( v.shape );
        m_color = v.color;
        return true;
    }
    catch( std::exception& ex ) { std::cerr << "view-points: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "view-points: unknown exception" << std::endl; }
    return false;
}

} } } // namespace snark { namespace graphics { namespace View {

#endif // SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_SHAPE_READER_H_
