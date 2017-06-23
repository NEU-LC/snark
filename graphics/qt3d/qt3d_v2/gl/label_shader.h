#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFramebufferObject>
#include <QPainter>
#include <Eigen/Core>
#include <memory>
#include <boost/array.hpp>

namespace snark { namespace graphics { namespace qt3d { namespace gl {

struct label_vertex
{
    /// x,y,z of the corner point 
    Eigen::Vector3f position;
    /// normalized offset of other corners 0/1
    Eigen::Vector2f offset;
    /// size of the image texture in pixels
    Eigen::Vector2f texture_size;
    label_vertex(float x,float y,float z,float ox,float oy,float width, float height);
};
    
class label : protected QOpenGLFunctions
{
    friend class label_shader;
public:
    label();
    virtual ~label();
    virtual void update()=0;
//     {
//         init();
//         resize(w,h);
//         update(x,y,z);
//         draw();
//     }
    /// update position vertex
    void update(float x,float y,float z);
    /// resize texture buffer
    void resize(int width,int height);
    /// resize and update position
//     void update(float x,float y,float z,int w,int h)
//     {
//         resize(w,h);
//         update(x,y,z);
//     }
protected:
    /// draw the label
    virtual void draw(QPainter& painter)=0;
    
    //no need to override these
    virtual void init();
    virtual void paint();
    void draw();
    virtual void destroy();
    
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
    std::vector<label_vertex> quad;
    
    std::unique_ptr<QOpenGLFramebufferObject> fbo;
    int width;
    int height;
};

class label_shader : protected QOpenGLFunctions
{
    friend class widget;
public:
    label_shader();
    virtual ~label_shader();
    void clear();   //delete labels
    void update();  //init and update all added labels
    std::vector<std::shared_ptr<label>> labels;

protected:
    //GL context should be set and voa bound for these functions by caller (i.e. gl_widget)
    virtual void init();    //create texture buffer
    virtual void paint(const QMatrix4x4& projection_matrix, const QSize& size);  //invoke glDraw*
    virtual void destroy();   //destroy buffer
protected:
    
    QOpenGLShaderProgram program;
    int projection_matrix_location;
    int sampler_location;
    int screen_size_location;
};
    
} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
