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

#ifndef SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_COLORED_H_
#define SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_COLORED_H_

#include <string>
#include <Qt3D/qcolor4ub.h>
#include "../../../../render/colour_map.h"
#include "../point_with_id.h"

namespace snark { namespace graphics { namespace view {

struct colored
{
    virtual ~colored() {}
    virtual QColor4ub color( const Eigen::Vector3d& point
                             , comma::uint32 id
                             , double scalar
                             , const QColor4ub& c ) const = 0;

};

class Fixed : public colored
{
    public:
        Fixed( const std::string& name );
        QColor4ub color( const Eigen::Vector3d& point
                         , comma::uint32 id
                         , double scalar
                         , const QColor4ub& c ) const;

    private:
        QColor4ub m_color;
};

struct ByHeight : public colored // todo: refactor and merge with byscalar
{
    ByHeight( double from
            , double to
            , const QColor4ub& from_color = QColor4ub( 255, 0, 0 )
            , const QColor4ub& to_color = QColor4ub( 0, 0, 255 )
            , bool cyclic = false
            , bool linear = true
            , bool sharp = false );

    double from, to, sum, diff, middle;
    QColor4ub from_color, to_color, average_color;
    bool cyclic, linear, sharp;

    QColor4ub color( const Eigen::Vector3d& point
                     , comma::uint32 id
                     , double scalar
                     , const QColor4ub& c ) const;
};

class ByScalar : public colored
{
    public:
        ByScalar( double from
                , double to
                , const QColor4ub& from_color
                , const QColor4ub& to_color );

        ByScalar( double from
                , double to
                , const snark::render::colour_map::values& map );

        QColor4ub color( const Eigen::Vector3d& point
                       , comma::uint32 id
                       , double scalar
                       , const QColor4ub& c ) const;

    protected:
        double from, to, diff;
        boost::optional< snark::render::colour_map::values > map;
        QColor4ub from_color;
        QColor4ub to_color;
};

class ById : public colored
{
    public:
        ById( const QColor4ub& backgroundcolor );

        ById( const QColor4ub& backgroundcolor
            , double from
            , double to );
        QColor4ub color( const Eigen::Vector3d& point
                       , comma::uint32 id
                       , double scalar
                       , const QColor4ub& c ) const;
    private:
        const QColor4ub m_background;
        bool m_hasScalar;
        double m_from;
        double m_diff;
};

struct ByRGB : public colored
{
    QColor4ub color( const Eigen::Vector3d& point
                     , comma::uint32 id
                     , double scalar
                     , const QColor4ub& c ) const;
};

colored* colorFromString( const std::string& s, const std::string& fields, const QColor4ub& backgroundcolor );

} } } // namespace snark { namespace graphics { namespace view {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_COLORED_H_*/
