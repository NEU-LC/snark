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

#include "./vertex_buffer.h"

namespace snark { namespace graphics { namespace qt3d {

vertex_buffer::vertex_buffer( std::size_t size )
    : read_index_( 0 )
    , write_index_( 0 )
    , read_size_( 0 )
    , write_end_( 0 )
    , write_size_( 0 )
    , buffer_size_( size )
    , block_( 0 )
    , points_( 2 * size )
    , color_( 2 * size )
{
}

void vertex_buffer::add_vertex( const QVector3D& point, const QColor4ub& color, unsigned int block )
{
    if( block != block_ ) { toggle(); }
    block_ = block;
    points_[ write_index_ + write_end_ ] = point;
    color_[ write_index_ + write_end_ ] = color;
    if( write_size_ < buffer_size_ ) { ++write_size_; }
    if( read_index_ == write_index_ ) { read_size_ = write_size_; }
    ++write_end_;
    if( write_end_ == buffer_size_ ) { write_end_ = 0; }
}

void vertex_buffer::toggle()
{
    if( write_size_ == 0 ) { return; }
    write_index_ = buffer_size_ - write_index_;
    if( read_index_ == write_index_ )
    {
        read_index_ = buffer_size_ - read_index_;
        read_size_ = write_end_;
    }
    write_end_ = 0;
    write_size_ = 0;
}

const QVector3DArray& vertex_buffer::points() const { return points_; }

const QArray< QColor4ub >& vertex_buffer::color() const { return color_; }

unsigned int vertex_buffer::size() const { return read_size_; }

unsigned int vertex_buffer::index() const { return read_index_; }



} } } // namespace snark { namespace graphics { namespace qt3d {
