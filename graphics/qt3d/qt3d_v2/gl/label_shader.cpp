#include "label_shader.h"
#include <QOpenGLPaintDevice>
#include <iostream>

namespace snark { namespace graphics { namespace qt3d { namespace gl {

/*    image/texture shader
/// draw image from texture in 3d projection
static const char *texture_shader_source = R"(
    #version 150
    in vec3 vertex;
    in vec2 offset;
    out vec2 textCoord;
    uniform mat4 projection_matrix;
    void main() {
       gl_Position = projection_matrix* vec4(vertex,1) ;
       textCoord = offset;
    }
)";
static const char *texture_fragment_source = R"(
    #version 150
    in vec2 textCoord;
    uniform sampler2D sampler;
    out highp vec4 frag_color;
    void main() {
       frag_color = texture2D(sampler,textCoord);
    }
)";
*/

static const char *shader_source = R"(
    #version 150
    in vec3 vertex;
    in vec2 offset;
    in vec2 texture_size;
    out vec2 textCoord;
    uniform mat4 projection_matrix;
    uniform vec2 screen_size;
    void main() {
        vec4 target=projection_matrix* vec4(vertex,1);
       gl_Position = vec4(target.x/target.z,target.y/target.z,0,1) + vec4(offset.x*texture_size.x/screen_size.x,offset.y*texture_size.y/screen_size.y,0,0);
       textCoord = offset;
    }
)";

static const char *fragment_source = R"(
    #version 150
    in vec2 textCoord;
    uniform sampler2D sampler;
    out highp vec4 frag_color;
    void main() {
       frag_color = texture2D(sampler,textCoord);
    }
)";

/*
    TODO
        texture frame buffer size to text size for text label
        remove test draw
        create subclass for text_label
        call draw in update? or resize? or add comment for correct usage form
*/    

label_vertex::label_vertex(float x,float y,float z,float ox, float oy,float w,float h) : position(x,y,z), offset(ox,oy), texture_size(w,h)
{
}

label_shader::label_shader()
{
//     labels.push_back(std::shared_ptr<label>(new label()));
    
}
label_shader::~label_shader()
{
    
}
void label_shader::clear()
{
    for(auto& i : labels) { i->destroy(); }
    labels.clear();
}
void label_shader::update()
{
    for(auto& i : labels) { i->update(); }
    
}
void label_shader::init()
{
//     std::cerr<<"label_shader::init"<<std::endl;
    initializeOpenGLFunctions();
    program.addShaderFromSourceCode( QOpenGLShader::Vertex, shader_source );
    program.addShaderFromSourceCode( QOpenGLShader::Fragment, fragment_source);
    program.bindAttributeLocation("vertex",0);
    program.bindAttributeLocation("offset",1);
    program.bindAttributeLocation("texture_size",2);
    program.link();
    program.bind();
    projection_matrix_location=program.uniformLocation("projection_matrix");
    sampler_location=program.uniformLocation("sampler");
    screen_size_location=program.uniformLocation("screen_size");
    
    program.release();
    
    for(auto& j : labels) { j->init(); }
}
void label_shader::paint(const QMatrix4x4& projection_matrix, const QSize& size)
{
    program.bind();
    program.setUniformValue(projection_matrix_location,projection_matrix);
    program.setUniformValue(sampler_location,0);
    program.setUniformValue(screen_size_location,QVector2D(size.width()/2,size.height()/2));
//     static int counter=0;
//     if(counter++<10)
//         std::cerr<<"label_shader::paint "<<size.width()/2<<", "<<size.height()/2<<std::endl;
    
    //?disable back-face culling
    //?disable depth test
    for(auto& j : labels) { j->paint(); }
    program.release();
}
void label_shader::destroy()   //destroy buffer
{
    for(auto& j : labels) { j->destroy(); }
}
label::label() : width(0),height(0) { } 
label::~label() { }
void label::init()
{
    initializeOpenGLFunctions();
    //shape init
    vao.create();
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
    vbo.bind();
    glEnableVertexAttribArray( 0 );
    glEnableVertexAttribArray( 1 );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( label_vertex ), reinterpret_cast<void *>( offsetof( label_vertex, position )));
    glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( label_vertex ), reinterpret_cast<void *>( offsetof( label_vertex, offset )));
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( label_vertex ), reinterpret_cast<void *>( offsetof( label_vertex, texture_size )));
    vbo.release();

    //sample test
//     resize(200,50);
//     update(1,2,1);
//     draw();
}
void label::resize(int w,int h)
{
    if(width!=w || height!=h)
    {
        width=w;
        height=h;
//         std::cerr<<"label::resize "<<width<<","<<height<<std::endl;
        QOpenGLFramebufferObjectFormat format;
        format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        fbo.reset(new QOpenGLFramebufferObject(width, height, format));
    }
}
// //updates quad buffer
// void label::update(float x,float y,float z,int w,int h)
// {
//     resize(w,h);
//     update(x,y,z);
// }
void label::update(float x,float y,float z)
{
//     std::cerr<<"label::update "<<width<<", "<<height<<std::endl;
    quad.clear();
    quad.push_back(label_vertex(x,y,z,0,0,width,height));
    quad.push_back(label_vertex(x,y,z,1,0,width,height));
    quad.push_back(label_vertex(x,y,z,1,1,width,height));
    quad.push_back(label_vertex(x,y,z,0,1,width,height));
    QOpenGLVertexArrayObject::Binder binder(&vao);
    vbo.bind();
    size_t size=quad.size()*sizeof(label_vertex);
    vbo.allocate(size);
    vbo.write(0,&quad[0],size);
    vbo.release();
}
void label::paint()
{
    if(fbo)
    {
//         static int counter=0;
//         if(counter++<2)
//             std::cerr<<"label_shader::paint"<<std::endl;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fbo->texture());
        
        QOpenGLVertexArrayObject::Binder binder(&vao);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glDrawArrays(GL_TRIANGLE_FAN,0,4);
    }
}

void label::draw()    //draw to texture
{
    if(fbo)
    {
//         std::cerr<<"label_shader::draw"<<std::endl;
        fbo->bind();
        
        QOpenGLPaintDevice paint_dev(width, height);
        QPainter painter(&paint_dev);
        draw(painter);
        fbo->release();
    }
}
/*
void label::draw(QPainter& painter)
{
    painter.setFont(QFont("System",32));
//     painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
    painter.fillRect(0,0,width,height,Qt::green);
//         painter.setBackgroundMode(Qt::OpaqueMode);
//         painter.setBackground(QBrush(Qt::green));
    painter.setPen( Qt::blue );
    painter.drawText(QRect(2,4,width, height), Qt::AlignTop, "Boo!");
    painter.end();
//         QImage image2 = fbo->toImage();
//         image2.save("/home/navid/data/snark/view-points/bla4.png");
}
*/
void label::destroy()
{
    fbo.release();
}


} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
