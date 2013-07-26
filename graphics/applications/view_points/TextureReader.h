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

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_TEXTURE_READER_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_TEXTURE_READER_H_

#include <vector>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "./Reader.h"
#include <Qt3D/qglbuilder.h>

namespace snark { namespace graphics { namespace View {

/// display an image as a texture, set its position from an input csv stream
class TextureReader : public Reader
{
    public:
        struct image_options
        {
            std::string filename;
            double width;
            double height;
            boost::optional< double > pixel_size;
            image_options() : width( 1 ), height( 1 ) {}
            image_options( const std::string& filename ) : filename( filename ), width( 1 ), height( 1 ) {}
            image_options( const std::string& filename, double pixel_size ) : filename( filename ), width( 0 ), height( 0 ), pixel_size( pixel_size ) {}
            image_options( const std::string& filename, double width, double height ) : filename( filename ), width( width ), height( height ) {}
        };

        TextureReader( QGLView& viewer, comma::csv::options& csv, const std::vector< image_options >& io );

        void start();
        std::size_t update( const Eigen::Vector3d& offset );
        const Eigen::Vector3d& somePoint() const;
        bool readOnce();
        void render( QGLPainter *painter );
        bool empty() const;

    protected:
        boost::scoped_ptr< comma::csv::input_stream< PointWithId > > m_stream;
        struct image_
        {
            QImage image;
            QGeometryData geometry;
            QGLBuilder builder;
            QGLSceneNode* node;
            QGLTexture2D texture;
            QGLMaterial material;

            image_( const image_options& o );
        };
        boost::ptr_vector< image_ > images_;
};

} } } // namespace snark { namespace graphics { namespace View {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_TEXTURE_READER_H_*/
