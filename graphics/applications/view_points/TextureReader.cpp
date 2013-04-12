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


/// @author Cedric Wohlleber

#include <Eigen/Core>
#include <snark/graphics/qt3d/rotation_matrix.h>
#include "./TextureReader.h"
#include "./Texture.h"

namespace snark { namespace graphics { namespace View {

/// constructor
/// @param viewer reference to the viewer
/// @param options csv options for the position input
/// @param file image filename
/// @param width image width in meters to be displayed in the scene
/// @param height image height in meters to be displayed in the scene
TextureReader::TextureReader( QGLView& viewer, comma::csv::options& options, const std::string& file, double width, double height )
    : Reader( viewer, options, 1, NULL, 1, "", QVector3D( 0, 1, 1 ) ), 
      m_file( file ),
      m_image( file.c_str() )
{
    m_texture.setImage( m_image );
    m_material.setTexture( &m_texture );

    QVector3D a( 0, 0, 0 );
    QVector3D b( width, 0, 0 );
    QVector3D c( width, height, 0 );
    QVector3D d( 0, height, 0 );
    QVector2D ta( 0, 0 );
    QVector2D tb( 1, 0 );
    QVector2D tc( 1, 1 );
    QVector2D td( 0, 1 );
    m_geometry.appendVertex( a, b, c, d );
    m_geometry.appendTexCoord(ta, tb, tc, td);
    m_builder.addQuads( m_geometry );
    m_node = m_builder.finalizedSceneNode();
    m_node->setMaterial( &m_material );
}

void TextureReader::start()
{        
    m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) );
}

void TextureReader::update( const Eigen::Vector3d& offset )
{
    updatePoint( offset );
}

bool TextureReader::empty() const
{
    return !m_point;
}

const Eigen::Vector3d& TextureReader::somePoint() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return *m_point;
}

void TextureReader::render( QGLPainter* painter )
{
    painter->setStandardEffect( QGL::FlatReplaceTexture2D );
    painter->modelViewMatrix().push();
    painter->modelViewMatrix().translate( m_translation );
    painter->modelViewMatrix().rotate( m_quaternion );
    
    m_node->draw(painter);
    painter->modelViewMatrix().pop();
}

bool TextureReader::readOnce()
{
    if( !m_stream ) // quick and dirty: handle named pipes
    {
        if( !m_istream() ) { return true; }
        m_stream.reset( new comma::csv::input_stream< PointWithId >( *m_istream(), options ) );
    }
    const PointWithId* p = m_stream->read();
    if( p == NULL ) { m_shutdown = true; return false; }
    boost::mutex::scoped_lock lock( m_mutex );
    m_point = p->point;
    m_orientation = p->orientation;
    return true;
}


} } } // namespace snark { namespace graphics { namespace View {
    
