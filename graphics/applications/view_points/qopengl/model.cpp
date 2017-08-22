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

/// @author Navid Pirmarzdashti

#include "model.h"
#include "comma/base/exception.h"

namespace snark { namespace graphics { namespace view { namespace qopengl {

void model::load(const std::string& file_name)
{
    mesh.reset(new snark::graphics::qopengl::model());
    //or move this to inside import
    if(!((snark::graphics::qopengl::model*)mesh.get())->import(file_name)) { COMMA_THROW( comma::exception, "failed to load model from: "<<file_name); }
}
void model::add_shaders(snark::graphics::qopengl::viewer_base* viewer_base)
{
    mesh_shader.reset(new snark::graphics::qopengl::mesh_shader);
    viewer_base->add_mesh_shader(mesh_shader);
}
void model::update_view()
{
    //if first time draw
    if(mesh_shader->meshes.empty())
    {
        mesh_shader->meshes.push_back(mesh);
//         ((snark::graphics::qopengl::model*)mesh.get())->make_mesh();
        //we could separate model from mesh by doing: mesh=model->make_mesh(); mesh_shader->add(mesh);
    }
    //update position and orientation
//     mesh_shader->visible=m_show; //this should go in reader
}
     
} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {
    
