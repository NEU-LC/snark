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
protected:
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    friend class widget;
public:
    shape( GLenum mode, float weight ); // GL_POINTS
    virtual ~shape() {}
    //this can only be called between begin_update and end_update
    //size: size of array e.g. update(v.data(),v.size())
    virtual void update( const vertex_t* data, std::size_t size );    //write data to internal buffer
    /// set true to draw, false to hide; call widget update to take effect
    bool visible;
protected:
    //GL context should be set and voa bound for these functions by caller (i.e. gl_widget)
    virtual void init();    //create buffer
    virtual void paint();  //invoke glDraw*
    virtual void destroy();   //destroy buffer
    void make_smooth();
    GLenum mode;
    float weight_;
    std::size_t size_;
private:
};

namespace shapes {
    
struct point : public shape
{
    point( float weight );
    void paint();
};

/// Two vertices per line. Vertices 0 and 1 are considered a line. Vertices 2 and 3 are considered a line, etc.
struct lines : public shape
{
    lines( float weight );
    void paint();
};

/// The adjacent vertices are considered lines. Vertice 0 and 1; then vertices 1 and 2, etc.
struct line_strip : public shape
{
    line_strip( float weight );
    void paint();
};

/// The adjacent vertices are considered lines (similar to line_strip) and then last vertice and first to close the loop
struct line_loop : public shape
{
    line_loop( float weight );
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
    
