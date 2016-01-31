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

/// @author Cedric Wohlleber

#include "TextureReader.h"

namespace snark { namespace graphics { namespace View {

TextureReader::image_::image_( const TextureReader::image_options& o ) : image( &o.filename[0] )
{
    texture.setImage( image );
    material.setTexture( &texture );
    comma::uint32 width = o.pixel_size ? *o.pixel_size * image.width() : o.width;
    comma::uint32 height = o.pixel_size ? *o.pixel_size * image.height() : o.height;
    if( width == 0 ) { COMMA_THROW( comma::exception, "got zero width for image " << o.filename ); }
    if( height == 0 ) { COMMA_THROW( comma::exception, "got zero height for image " << o.filename ); }
    QVector3D a( 0, 0, 0 );
    QVector3D b( width, 0, 0 );
    QVector3D c( width, height, 0 );
    QVector3D d( 0, height, 0 );
    QVector2D ta( 0, 0 );
    QVector2D tb( 1, 0 );
    QVector2D tc( 1, 1 );
    QVector2D td( 0, 1 );
    geometry.appendVertex( a, b, c, d );
    geometry.appendTexCoord(ta, tb, tc, td);
    builder.addQuads( geometry );
    node = builder.finalizedSceneNode();
    node->setMaterial( &material );
}

TextureReader::TextureReader( QGLView& viewer
                            , comma::csv::options& csv
                            , const std::vector< image_options >& io )
    : Reader( viewer, reader_parameters( csv ), NULL, "", QVector3D( 0, 1, 1 ) )
{
    for( unsigned int i = 0; i < io.size(); ++i ) { images_.push_back( new image_( io[i] ) ); }
}

void TextureReader::start()
{
    m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) );
}

std::size_t TextureReader::update( const Eigen::Vector3d& offset )
{
    return updatePoint( offset ) ? 1 : 0;
}

bool TextureReader::empty() const { return !m_point; }

const Eigen::Vector3d& TextureReader::somePoint() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return *m_point;
}

void TextureReader::render( QGLPainter* painter )
{
    if( !m_point ) { return; }
    comma::uint32 id = id_;
    if( id >= images_.size() ) { return; }
    painter->setStandardEffect( QGL::FlatReplaceTexture2D );
    painter->modelViewMatrix().push();
    painter->modelViewMatrix().translate( m_translation - m_offset );
    painter->modelViewMatrix().rotate( m_quaternion );
    images_[id].node->draw( painter );
    painter->modelViewMatrix().pop();
}

bool TextureReader::read_once()
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
    id_ = p->id;
    updated_ = true;
    return true;
}


} } } // namespace snark { namespace graphics { namespace View {

