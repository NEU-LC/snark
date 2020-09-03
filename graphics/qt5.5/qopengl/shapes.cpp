// Copyright (c) 2016 The University of Sydney

/// @author Navid Pirmarzdashti

#include "shapes.h"
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {
    
shape::shape(GLenum mode):mode(mode),size_(0)
{
    
}
shape::~shape()
{
}
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


void shape::update( const vertex_t* data, std::size_t size )
{
    std::cerr << "--> shapes: a: update: size: " << size << std::endl;
    QOpenGLVertexArrayObject::Binder binder( &vao );
    vbo.bind();
    vbo.allocate( size * sizeof( vertex_t ) );
    vbo.write( 0, data, size * sizeof( vertex_t ) );
    vbo.release();
    size_ = size;
}
void shape::paint()
{
    QOpenGLVertexArrayObject::Binder binder( &vao );
    glDrawArrays( mode, 0, size_ );
}
void shape::destroy()
{
    vbo.destroy();
}
namespace shapes {
    
point::point( float point_size ) : shape( GL_POINTS ), point_size( point_size ) {}

void point::paint()
{
    //disable GL_PROGRAM_POINT_SIZE
    glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
    if( point_size != 1 ) { glEnable( GL_POINT_SMOOTH ); } //circular point, otherwise draws square points
    glPointSize( point_size );
    shape::paint();
}

lines::lines( float line_width ) : shape( GL_LINES ),line_width( line_width ) {}

void lines::paint()
{
    if( line_width != 1 )
    {
        glLineWidth( line_width );
        glEnable( GL_LINE_SMOOTH );
    }
    shape::paint();

// //if not supported by vga
// GLfloat lineWidthRange[2];
// glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lineWidthRange);
}

line_strip::line_strip( float line_width ) : shape( GL_LINE_STRIP ),line_width( line_width ) {}

void line_strip::paint()
{
    if( line_width != 1 )
    {
        glLineWidth( line_width );
        glEnable( GL_LINE_SMOOTH );
    }
    shape::paint();
}

line_loop::line_loop(float line_width) : shape(GL_LINE_LOOP),line_width(line_width) { }

void line_loop::paint()
{
    if(line_width!=1)
    {
        glLineWidth(line_width);
        glEnable(GL_LINE_SMOOTH);
    }
    shape::paint();
}

triangles::triangles(bool fill) : shape(GL_TRIANGLES),fill(fill) { }
void triangles::paint()
{
    glPolygonMode(GL_FRONT_AND_BACK,fill ? GL_FILL : GL_LINE);
    shape::paint();
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
}

} // namespace shapes {

} } } // namespace snark { namespace graphics { namespace qopengl {
    
