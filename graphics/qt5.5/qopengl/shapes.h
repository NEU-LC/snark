// Copyright (c) 2016 The University of Sydney

/// @author Navid Pirmarzdashti

#pragma once

#include "types.h"
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>

namespace snark { namespace graphics { namespace qopengl {

// TODO refactor shape shader into separate class out of widget, similar to label_shader
// class shape_shader
// {
// public:
//     std::vector<std::shared_ptr<shape>> shapes;
//     //set GL context and bind voa, ?optionaly synchronize with paint
//     virtual void begin_update()=0;
//     virtual void end_update()=0;
// protected:
//     void init();
//     void paint();
//     void destroy();
// };

class shape : protected QOpenGLFunctions
{
public:
    shape( GLenum mode, float weight = 1, unsigned int size = 1 ); // GL_POINTS
    
    virtual ~shape() {}
    
    //this can only be called between begin_update and end_update
    //size: size of array e.g. update(v.data(),v.size())
    virtual void update( const vertex_t* vertices, std::size_t vertex_count );
    
    /// set true to draw, false to hide; call widget update to take effect
    bool visible;
    
protected:
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    friend class widget;
    GLenum mode;
    float weight_;
    std::size_t size_;
    std::size_t vertex_count_;
    
    //GL context should be set and voa bound for these functions by caller (i.e. gl_widget)
    virtual void init();    //create buffer
    virtual void paint();  //invoke glDraw*
    virtual void destroy();   //destroy buffer
    void make_smooth();
};

namespace shapes {
    
struct point : public shape
{
    point( float weight = 1 );
    void paint();
};

/// Two vertices per line. Vertices 0 and 1 are considered a line. Vertices 2 and 3 are considered a line, etc.
struct lines : public shape
{
    lines( float weight = 1 );
    void paint();
};

/// The adjacent vertices are considered lines. Vertice 0 and 1; then vertices 1 and 2, etc.
struct line_strip : public shape
{
    line_strip( float weight = 1, unsigned int size = 1 );
    void paint();
};

/// The adjacent vertices are considered lines (similar to line_strip) and then last vertice and first to close the loop
struct line_loop : public shape
{
    line_loop( float weight = 1, unsigned int size = 1 );
    void paint();
};

struct triangles : public shape
{
    bool fill;
    
    triangles( bool fill = false );
    void paint();
};

} // namespace shapes {

} } } // namespace snark { namespace graphics { namespace qopengl {
    
