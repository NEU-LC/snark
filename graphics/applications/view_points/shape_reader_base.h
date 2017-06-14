// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

#pragma once
#include <Eigen/Core>
#include "../../../math/interval.h"
#include "reader.h"
#include "types.h"
#include "../../block_buffer.h"

namespace snark { namespace graphics { namespace view {

class shape_reader_base : public Reader
{
public:
    void add_vertex(const vertex_t& v, unsigned int block);
    void add_label(const label_t& l, unsigned int block);
    void extent_hull(const snark::math::closed_interval< float, 3 >& x);
    void extent_hull(const Eigen::Vector3f& p);
    const Eigen::Vector3d& offset();
protected:
    shape_reader_base(const reader_parameters& params, colored* c, const std::string& label, std::size_t shape_size);
    block_buffer< vertex_t > buffer_;
    block_buffer< label_t > labels_;
};

shape_reader_base::shape_reader_base(const reader_parameters& params, colored* c, const std::string& label, std::size_t shape_size)
    : Reader( params, c, label )
    , buffer_( size * shape_size )
    , labels_( size )
{
    
}
inline void shape_reader_base::add_vertex(const vertex_t& v, unsigned int block)
{
    buffer_.add(v,block);
}
inline void shape_reader_base::add_label(const label_t& l, unsigned int block)
{
    labels_.add(l,block);
}
inline void shape_reader_base::extent_hull(const snark::math::closed_interval< float, 3 >& x)
{
    m_extents = m_extents ? m_extents->hull( x ) : x;
}
inline void shape_reader_base::extent_hull(const Eigen::Vector3f& p)
{
    m_extents = m_extents ? m_extents->hull( p ) : snark::math::closed_interval< float, 3 >(p);
}
inline const Eigen::Vector3d& shape_reader_base::offset()
{
    return m_offset;
}

} } } // namespace snark { namespace graphics { namespace view {
    
