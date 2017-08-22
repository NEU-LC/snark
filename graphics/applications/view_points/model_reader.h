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

#pragma once

#include "reader.h"

#if Qt3D_VERSION==1
#include "qt3d_v1/model.h"
#include "qt3d_v1/ply_loader.h"

#elif Qt3D_VERSION==2
#include "qopengl/model.h"
#endif

namespace snark { namespace graphics { namespace view {

struct model_options
{
    std::string filename;
    bool flip;
    double scale;
    model_options() : flip( false ), scale( 1.0 ) {}
};

/// display 3d models ( obj or 3ds ), set its position from an input csv stream
class model_reader : public Reader
{
    public:
        model_reader( const reader_parameters& params
                   , const model_options& options
                   , colored* c
                   , const std::string& label );

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
        boost::scoped_ptr< comma::csv::input_stream< PointWithId > > m_stream;
        boost::scoped_ptr< comma::csv::passed< PointWithId > > m_passed;
        
#if Qt3D_VERSION==1
        qt3d_v1::model model;
        boost::optional< PlyLoader > m_plyLoader;
        
#elif Qt3D_VERSION==2
        qopengl::model model;
#endif
        const std::string m_file;
        bool m_flip;
        double scale_;
        const colored* colored_;
};

} } } // namespace snark { namespace graphics { namespace view {
    
namespace comma { namespace visiting {

template <> struct traits< snark::graphics::view::model_options >
{
    template < typename Key, class Visitor > static void visit( Key, snark::graphics::view::model_options& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "flip", p.flip );
        v.apply( "scale", p.scale );
    }

    template < typename Key, class Visitor > static void visit( Key, const snark::graphics::view::model_options& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "flip", p.flip );
        v.apply( "scale", p.scale );
    }
};

} } // namespace comma { namespace visiting {
   
