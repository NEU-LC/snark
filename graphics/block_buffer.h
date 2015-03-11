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

/// @author Vsevolod Vlaskine

#ifndef SNARK_GRAPHICS_BLOCK_BUFFER_H_
#define SNARK_GRAPHICS_BLOCK_BUFFER_H_

#include <vector>
#include <boost/array.hpp>

namespace snark { namespace graphics {

/// circular double buffer accumulating values blockwise
/// until the block is complete, then it becomes available for reading
///
/// @todo reuse for qt3d::vertex_buffer
template < typename T, typename Storage = std::vector< T > >
class block_buffer
{
    public:
        /// constructor
        block_buffer( std::size_t size );

        /// add value
        /// @param point vertex coordinates
        /// @param color vertex color
        /// @param block id of a block of values (e.g. vertices); on change of block id, double buffer toggles
        void add( const T& point, unsigned int block = 0 );

        /// toggle double buffer, so that the buffer currently accumulating (the "write" buffer)
        /// becomes available for reading
        ///
        /// adding vertices after this call may produce strange results
        ///
        /// @note should be done on the last block, since otherwise
        ///       there is no way to determine that the last block has
        ///       been finished and thus the last block will not be
        ///       properly visualized
        void toggle();

        /// return current buffer
        const Storage& values() const;

        /// return current size of the buffer that is ready for reading
        unsigned int size() const;

        /// return current offset in the buffer that is ready for reading
        unsigned int index() const;
        
        /// return true, if data has changed since seen last time
        bool changed() const;
        
        /// reset changed flag
        void mark_seen();

    protected:
        unsigned int read_block_;
        unsigned int write_block_;
        unsigned int read_size_;
        unsigned int write_end_;
        unsigned int write_size_;
        unsigned int block_;
        bool changed_;
        boost::array< Storage, 2 > values_;
};

template < typename T, typename Storage >
inline block_buffer< T, Storage >::block_buffer( std::size_t size )
    : read_block_( 0 )
    , write_block_( 0 )
    , read_size_( 0 )
    , write_end_( 0 )
    , write_size_( 0 )
    , block_( 0 )
    , changed_( false )
{
    values_[0].resize( size );
    values_[1].resize( size );
}

template < typename T, typename Storage >
inline void block_buffer< T, Storage >::add( const T& t, unsigned int block )
{
    if( block != block_ ) { toggle(); }
    block_ = block;
    values_[write_block_][write_end_] = t;
    if( write_size_ < values_[0].size() ) { ++write_size_; }
    if( read_block_ == write_block_ ) { read_size_ = write_size_; changed_ = true; }
    ++write_end_;
    if( write_end_ == values_[0].size() ) { write_end_ = 0; }
}

template < typename T, typename Storage >
inline void block_buffer< T, Storage >::toggle()
{
    changed_ = true;
    if( write_size_ == 0 ) { return; }
    write_block_ = 1 - write_block_;
    if( read_block_ == write_block_ )
    {
        read_block_ = 1 - read_block_;
        read_size_ = write_size_;
    }
    write_end_ = 0;
    write_size_ = 0;
}

template < typename T, typename Storage >
inline const Storage& block_buffer< T, Storage >::values() const { return values_[read_block_]; }

template < typename T, typename Storage >
inline unsigned int block_buffer< T, Storage >::size() const { return read_size_; }

template < typename T, typename Storage >
inline unsigned int block_buffer< T, Storage >::index() const { return read_block_; }

template < typename T, typename Storage >
inline bool block_buffer< T, Storage >::changed() const { return changed_; }

template < typename T, typename Storage >
inline void block_buffer< T, Storage >::mark_seen() { changed_ = false; }

} } // namespace snark { namespace graphics {

#endif // SNARK_GRAPHICS_BLOCK_BUFFER_H_
