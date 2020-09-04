// Copyright (c) 2016 The University of Sydney

/// @author Navid Pirmarzdashti

#include "shapes.h"
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {
    
shape::shape( GLenum mode, float weight, unsigned int size ): mode( mode ), weight_( weight ), size_( size ), vertex_count_( 0 ) {}

void shape::init()
{
    initializeOpenGLFunctions();
    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    vao.create();
    QOpenGLVertexArrayObject::Binder binder( &vao );
    vbo.create();
    vbo.setUsagePattern( QOpenGLBuffer::StreamDraw );
    vbo.bind();
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vertex_t ), reinterpret_cast<void *>( offsetof( vertex_t, position )));
    glVertexAttribPointer( 1, 4, GL_FLOAT, GL_FALSE, sizeof( vertex_t ), reinterpret_cast<void *>( offsetof( vertex_t, color )));
    vbo.release();
}

void shape::update( const vertex_t* vertices, std::size_t vertex_count )
{
    QOpenGLVertexArrayObject::Binder binder( &vao );
    vbo.bind();
    vbo.allocate( vertex_count * sizeof( vertex_t ) );
    vbo.write( 0, vertices, vertex_count * sizeof( vertex_t ) );
    vbo.release();
    vertex_count_ = vertex_count;
}

void shape::make_smooth()
{
    if( weight_ == 1 ) { return; }
    glLineWidth( weight_ );
    glEnable( GL_LINE_SMOOTH );
}

void shape::paint()
{
    QOpenGLVertexArrayObject::Binder binder( &vao );
    if( size_ == 1 )
    {
        glDrawArrays( mode, 0, vertex_count_ );
    }
    else
    {
        for( unsigned int i = 0; i < vertex_count_; i += size_ ) { glDrawArrays( mode, i, size_ ); } // quick and dirty
    }
}

void shape::destroy() { vbo.destroy(); }

namespace shapes {
    
point::point( float weight ) : shape( GL_POINTS, weight ) {}

void point::paint()
{
    glHint( GL_POINT_SMOOTH_HINT, GL_NICEST ); // disable GL_PROGRAM_POINT_SIZE
    if( weight_ > 1 ) { glEnable( GL_POINT_SMOOTH ); } // circular point, otherwise draws square points
    glPointSize( weight_ );
    shape::paint();
    if( weight_ > 1 ) { glDisable( GL_POINT_SMOOTH ); }
}

lines::lines( float weight ) : shape( GL_LINES, weight ) {}

void lines::paint()
{
    shape::make_smooth();
    shape::paint();

// //if not supported by vga
// GLfloat lineWidthRange[2];
// glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lineWidthRange);
}

line_strip::line_strip( float weight, unsigned int size ) : shape( GL_LINE_STRIP, weight, size ) {}

void line_strip::paint()
{
    shape::make_smooth();
    shape::paint();
}

line_loop::line_loop( float weight, unsigned int size ) : shape( GL_LINE_LOOP, weight, size ) { }

void line_loop::paint()
{
    shape::make_smooth();
    shape::paint();
}

triangles::triangles( bool fill ): shape( GL_TRIANGLES, 1. ), fill( fill ) {}

void triangles::paint()
{
    glPolygonMode( GL_FRONT_AND_BACK,fill ? GL_FILL : GL_LINE );
    shape::paint();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}

} // namespace shapes {

} } } // namespace snark { namespace graphics { namespace qopengl {
    
