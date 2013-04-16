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

#include "./vertex_buffer.h"


namespace snark { namespace graphics { namespace qt3d {

vertex_buffer::vertex_buffer ( std::size_t size ):
    m_readIndex( 0 ),
    m_writeIndex( 0 ),
    m_readSize( 0 ),
    m_writeSize( 0 ),
    m_bufferSize( size ),
    m_block( 0 )
{
    m_points.resize( 2 * size );
    m_color.resize( 2 * size );
}

void vertex_buffer::addVertex ( const QVector3D& point, const QColor4ub& color, unsigned int block )
{
    if( block != m_block )
    {
        m_block = block;

        m_writeIndex += m_bufferSize;
        m_writeIndex %= 2 * m_bufferSize;
        if( m_readIndex == m_writeIndex )
        {
            m_readIndex+= m_bufferSize;
            m_readIndex %= 2 * m_bufferSize;
            m_readSize = m_writeSize;
        }
        m_writeSize = 0;
    }
    m_points[ m_writeIndex + m_writeSize ] = point;
    m_color[ m_writeIndex + m_writeSize ] = color;
    m_writeSize++;
    if( block == 0 )
    {
        m_readSize++;
    }
    if( m_writeSize > m_bufferSize )
    {
        m_writeSize = 0;
        m_readSize = m_bufferSize;
    }
}

const QVector3DArray& vertex_buffer::points() const
{
    return m_points;
}

const QArray< QColor4ub >& vertex_buffer::color() const
{
    return m_color;
}

const unsigned int vertex_buffer::size() const
{
    return m_readSize;
}

const unsigned int vertex_buffer::index() const
{
    return m_readIndex;
}


    
} } } // namespace snark { namespace graphics { namespace qt3d {
