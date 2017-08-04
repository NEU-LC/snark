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

#include "reader.h"

#if Qt3D_VERSION==2
#include "../../qt5.5/qopengl/textures.h"
#endif

namespace snark { namespace graphics { namespace view {

struct image_options
{
    std::string filename;
    Eigen::Vector2d size;
    image_options();
    image_options(const std::string& filename);
    image_options(const std::string& filename, double pixel_size );
    image_options(const std::string& filename, double width, double height);
};

#if Qt3D_VERSION==2
//TODO move this to qt3d_v2?
struct image
{
    image(const image_options& options);
    void update_view(const Eigen::Vector3d& position,const Eigen::Vector3d& orientation);
    snark::math::closed_interval<float,3> extents() const;
    
    Eigen::Vector2d size;
    std::shared_ptr<snark::graphics::qopengl::textures::image> image_texture;
};
typedef image image_t;
#endif
    
struct image_reader : public Reader
{
    image_reader(const reader_parameters& params, const std::vector<image_options>& io);

    void start();
    std::size_t update( const Eigen::Vector3d& offset );
    const Eigen::Vector3d& somePoint() const;
    bool read_once();
    bool empty() const;
#if Qt3D_VERSION==1
    void render( Viewer& viewer, QGLPainter *painter );
#elif Qt3D_VERSION==2
    virtual void add_shaders(snark::graphics::qopengl::viewer_base* viewer_base);
    virtual void update_view();
#endif
        
    
protected:
    boost::scoped_ptr< comma::csv::input_stream< PointWithId > > stream;
    boost::scoped_ptr< comma::csv::passed< PointWithId > > passed;
#if Qt3D_VERSION==2
    std::shared_ptr<snark::graphics::qopengl::texture_shader> texture_shader;
    std::vector<std::unique_ptr<image_t>> images;
#endif
};
    
} } } // namespace snark { namespace graphics { namespace view {
    
