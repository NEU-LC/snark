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

#include "./Texture.h"

namespace snark { namespace graphics { namespace View {

Quad::Quad ( const QImage& image )
{
    QVector3D a( 0, 0, 0 );
    QVector3D b( image.width(), 0, 0 );
    QVector3D c( image.width(), image.height(), 0 );
    QVector3D d( 0, image.height(), 0 );
    QVector2D ta( 0, 0 );
    QVector2D tb( 1, 0 );
    QVector2D tc( 1, 1 );
    QVector2D td( 0, 1 );
    m_geometry.appendVertex( a, b, c, d );
    m_geometry.appendTexCoord(ta, tb, tc, td);
    m_builder.addQuads( m_geometry );
    m_sceneNode = m_builder.finalizedSceneNode();
    m_texture.setImage( image );
    m_material.setTexture( &m_texture );
    m_sceneNode->setMaterial( &m_material );
}


QGLSceneNode* Quad::node() const
{
    return m_sceneNode;
}


    
Texture::Texture ( const QString& string, const QColor4ub& color )
{
    QFont font( "helvetica", 64 );
    QFontMetrics metrics( font );
    QRect rect = metrics.boundingRect(string);

    m_image = QImage( rect.size(), QImage::Format_ARGB32 );
    m_image.fill( Qt::transparent );
    QPainter p2( &m_image );
    p2.setRenderHint(QPainter::Antialiasing);
    p2.setFont( font );
    p2.setPen( color.toColor() );
    p2.drawText( -rect.x(), -rect.y(), string );
    p2.end();
}

void Texture::draw ( QGLPainter* painter )
{
    Quad quad( m_image );
    painter->setStandardEffect( QGL::FlatReplaceTexture2D );
    glDepthMask(GL_FALSE);
//     glEnable(GL_BLEND);
    quad.node()->draw( painter );
//     glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}



} } } // namespace snark { namespace graphics

