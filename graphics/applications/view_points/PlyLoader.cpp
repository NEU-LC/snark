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

#include <fstream>
#include <cstring>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/csv/options.h>
#include <comma/string/split.h>
#include <snark/visiting/eigen.h>
#include "./PlyLoader.h"

namespace snark { namespace graphics { namespace View {

struct ply_vertex
{
    Eigen::Vector3d point;
    Eigen::Vector3d normal; // todo: make normals boost::optional
    QColor4ub color;
    ply_vertex() : point( 0, 0, 0 ), normal( 0, 0, 0 ), color( 255, 255, 255, 255 ) {} //ply_vertex() : point( 0, 0, 0 ), normal( 0, 0, 0 ), r( 0 ), g( 0 ), b( 0 ), a( 255 ) {}
};

} } } // namespace snark { namespace graphics { namespace View {

namespace comma { namespace visiting {

template <> struct traits< snark::graphics::View::ply_vertex >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::graphics::View::ply_vertex& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
        unsigned int i = p.color.red();
        v.apply( "r", i );
        p.color.setRed( i );
        i = p.color.green();
        v.apply( "g", i );
        p.color.setGreen( i );
        i = p.color.blue();
        v.apply( "b", i );
        p.color.setBlue( i );
        i = p.color.alpha();
        v.apply( "a", i );
        p.color.setAlpha( i );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::graphics::View::ply_vertex& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
        v.apply( "r", p.color.red() );
        v.apply( "g", p.color.green() );
        v.apply( "b", p.color.blue() );
        v.apply( "a", p.color.alpha() );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace graphics { namespace View {

PlyLoader::PlyLoader( const std::string& file, boost::optional< QColor4ub > color ) : color_( color )
{
    std::ifstream stream( file.c_str() );
    char line[255];
    stream.getline( line, 255 );
    if( std::strcmp(line, "ply" ) != 0 ) { COMMA_THROW( comma::exception, "expected ply file; got \"" << line << "\" in " << file ); }
    std::string vertex = "element vertex";
    std::string face = "element face";
    unsigned int numVertex = 0;
    unsigned int numFace = 0;
    bool has_normals = false;

    std::vector< std::string > fields;
    while( std::strcmp(line, "end_header" ) != 0 )
    {
        stream.getline( line, 255 );
        if( std::memcmp( line, &vertex[0], vertex.size() ) == 0 )
        {
            std::string num( line + vertex.size() + 1 );
            numVertex = boost::lexical_cast< unsigned int >( num );
        }
        else if( std::memcmp( line, &face[0], face.size() ) == 0 )
        {
            std::string num( line + face.size() + 1 );
            numFace = boost::lexical_cast< unsigned int >( num );
        }
        else if( std::strcmp(line, "property float x" ) == 0 ) { fields.push_back( "point/x" ); }
        else if( std::strcmp(line, "property float y" ) == 0 ) { fields.push_back( "point/y" ); }
        else if( std::strcmp(line, "property float z" ) == 0 ) { fields.push_back( "point/z" ); }
        else if( std::strcmp(line, "property float nx" ) == 0 ) { fields.push_back( "normal/x" ); has_normals = true; }
        else if( std::strcmp(line, "property float ny" ) == 0 ) { fields.push_back( "normal/y" ); }
        else if( std::strcmp(line, "property float nz" ) == 0 ) { fields.push_back( "normal/z" ); }
        else if( std::strcmp(line, "property uchar red" ) == 0 ) { fields.push_back( "r" ); }
        else if( std::strcmp(line, "property uchar green" ) == 0 ) { fields.push_back( "g" ); }
        else if( std::strcmp(line, "property uchar blue" ) == 0 ) { fields.push_back( "b" ); }
        else if( std::strcmp(line, "property uchar alpha" ) == 0 ) { fields.push_back( "a" ); }
    }
    comma::csv::options csv;
    csv.fields = comma::join( fields, ',' );
    csv.full_xpath = true;
    csv.delimiter = ' ';
    comma::csv::ascii< ply_vertex > ascii( csv );
    QGeometryData geometry;
    QArray< QVector3D > vertices;
    QArray< QColor4ub > colors;

    for( unsigned int i = 0; i < numVertex; i++ )
    {
        std::string s;
        while( s.empty() && !stream.eof() ) { std::getline( stream, s ); }
        ply_vertex v;
        if( color_ ) { v.color = *color_; } // quick and dirty
        ascii.get( v, s );
        if( numFace > 0 )
        {
            geometry.appendVertex( QVector3D( v.point.x(), v.point.y(), v.point.z() ) );
            if( has_normals ) { geometry.appendNormal( QVector3D( v.normal.x(), v.normal.y(), v.normal.z() ) ); }
            geometry.appendColor( v.color );
        }
        else
        {
            vertices.append( QVector3D( v.point.x(), v.point.y(), v.point.z() ) );
            // todo: normals?
            colors.append( v.color );
        }
    }
    if( numFace > 0 )
    {
        unsigned int vertices_per_face = 0;
        for( unsigned int i = 0; i < numFace; i++ ) // quick and dirty
        {
            std::string s;
            while( s.empty() && !stream.eof() ) { std::getline( stream, s ); }
            std::vector< std::string > v = comma::split( s, ' ' );
            unsigned int n = boost::lexical_cast< unsigned int >( v[0] );
            if( ( n + 1 ) != v.size() ) { COMMA_THROW( comma::exception, "invalid line \"" << s << "\"" ); }
            if( vertices_per_face && n != vertices_per_face ) { COMMA_THROW( comma::exception, "only equal number of vertices per face supported" ); }
            vertices_per_face = n;
            QGL::IndexArray indices;
            switch( vertices_per_face )
            {
                case 3:
                    for( unsigned int i = 1; i < n; ++i ) { indices.append( boost::lexical_cast< unsigned int >( v[i] ) ); }
                    break;
                case 4: // quick and dirty for now: triangulate
                    boost::array< unsigned int, 4 > a;
                    for( unsigned int i = 1; i < 4; ++i ) { a[i] = boost::lexical_cast< unsigned int >( v[i] ); }
                    indices.append( a[0] );
                    indices.append( a[1] );
                    indices.append( a[2] );
                    indices.append( a[0] );
                    indices.append( a[2] );
                    indices.append( a[3] );
                    break;
                default: // never here
                    break;
            }
            geometry.appendIndices( indices );
        }
        QGLBuilder builder;
        builder.addTriangles( geometry );
        //switch( vertices_per_face )
        //{
        //    case 3: builder.addTriangles( geometry ); break;
        //   case 4: builder.addQuads( geometry ); break;
        //  default: COMMA_THROW( comma::exception, "only triangles and quads supported; but got " << vertices_per_face << " vertices per face" );
        //}
        m_sceneNode = builder.finalizedSceneNode();
    }
    else
    {
        m_vertices.addAttribute( QGL::Position, vertices );
        // todo: normals?
        m_vertices.addAttribute( QGL::Color, colors );
        m_vertices.upload();
        m_sceneNode = NULL;
    }
    stream.close();
}

void PlyLoader::draw ( QGLPainter* painter )
{
    painter->setStandardEffect(QGL::FlatPerVertexColor);
    if( m_sceneNode != NULL )
    {
        m_sceneNode->draw( painter );
    }
    else
    {
        painter->clearAttributes();
        // use vbo buffers, will get uploaded to the GL server only once
        painter->setVertexBundle( m_vertices );
        painter->draw( QGL::Points, m_vertices.vertexCount() );
    }
}


} } }
