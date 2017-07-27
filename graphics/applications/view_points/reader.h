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


/// @author Vsevolod Vlaskine, Cedric Wohlleber

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_READER_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_READER_H_

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include "colored.h"
#include "../../../math/interval.h"
#if Qt3D_VERSION==1
#include <Qt3D/qglview.h>
#else
#include <QQuaternion>
#include "../../qt5.5/qopengl/shapes.h"
#include "../../qt5.5/qopengl/labels.h"
#include <memory>
#endif
#include "types.h"

namespace snark { namespace graphics { namespace view {

#if Qt3D_VERSION==1
class Viewer;
#elif Qt3D_VERSION==2
namespace qt3d_v2 {
class viewer;
}
#endif

struct reader_parameters
{
    comma::csv::options options;
    std::string title;
    std::size_t size;
    unsigned int point_size;
    bool pass_through;
    bool fill; // quick and dirty
    std::string labels; //currently used for axis labels e.g. "x:y:z"
    double length;  //currently used for axis length
    bool has_color; //currently used for shape=axis, when false use three different color for xyz axises

    reader_parameters( const comma::csv::options& options
                     , const std::string& title
                     , std::size_t size
                     , unsigned int point_size
                     , bool pass_through
                     , bool fill 
                     ,std::string labels
                     ,double length
                     ,bool has_color)
        : options( options )
        , title( title )
        , size( size )
        , point_size( point_size )
        , pass_through( pass_through )
        , fill( fill )
        ,labels(labels)
        ,length(length)
        ,has_color(has_color)
    {}
};

class Reader : public reader_parameters
{
    public:
        Reader( const reader_parameters& params
              , colored* c
              , const std::string& label
              , const Eigen::Vector3d& offset = Eigen::Vector3d( 0, 0, 0 ) );

        virtual ~Reader() {}

        virtual void start() = 0;
        virtual std::size_t update( const Eigen::Vector3d& offset ) = 0;
        virtual const Eigen::Vector3d& somePoint() const = 0;
        virtual bool read_once() = 0;
#if Qt3D_VERSION==1
        virtual void render( Viewer& viewer, QGLPainter *painter ) = 0;
#endif
        virtual bool empty() const = 0;

        void show( bool s );
        bool show() const;
        bool isShutdown() const;
        bool isStdIn() const { return m_isStdIn; }
        void shutdown();
        void read();

#if Qt3D_VERSION==1
        friend class Viewer;
#elif Qt3D_VERSION==2
    friend class controller;
public:
    virtual std::shared_ptr<snark::graphics::qopengl::shape> make_shape()=0;
    virtual std::shared_ptr<snark::graphics::qopengl::label_shader> make_label_shader()=0;
    virtual void update_shape()=0;
    virtual void update_labels()=0;
#endif
             
    protected:
        bool updatePoint( const Eigen::Vector3d& offset );

        boost::optional< snark::math::closed_interval< float, 3 > > m_extents;
        unsigned int m_num_points;
        boost::scoped_ptr< colored > m_colored;
        bool m_shutdown;
        bool m_isStdIn;
        bool m_show;
        comma::io::istream m_istream;
        std::ostream* m_pass_through;
        boost::scoped_ptr< boost::thread > m_thread;
        mutable boost::mutex m_mutex;
        boost::optional< Eigen::Vector3d > m_point;
        bool updated_; // quick and dirty, terrible
        boost::optional< Eigen::Vector3d > m_orientation;
        comma::uint32 id_; // todo: quick and dirty; replace m_point, m_orientation, etc with PointWithId point_;
        color_t m_color;
        std::string m_label;
        Eigen::Vector3d m_translation;
        Eigen::Vector3d m_offset;
        QQuaternion m_quaternion;
};

} } } // namespace snark { namespace graphics { namespace view {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_READER_H_*/
