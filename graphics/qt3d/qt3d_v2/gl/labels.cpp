#include "labels.h"
#include <QPainter>
#include <QOpenGLPaintDevice>
#include <iostream>

namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
text_label::text_label(Eigen::Vector3d position, std::string text,color_t color) : position(position), text(text), color(color), width(1), height(1)
{
}

void text_label::update()
{
    init();
    resize(width,height);
    //calculate width and height from text using font
    if(fbo)
    {
        fbo->bind();
        QOpenGLPaintDevice paint_dev(width, height);
        QPainter painter(&paint_dev);
        painter.setFont(QFont("System",16));
        QRect rect=painter.boundingRect(QRect(2,4,1000, 1000), Qt::AlignTop, QString(text.data()));
        width=rect.width();
        height=rect.height();
        fbo->release();
    }
//     else
//         std::cerr<<"no fbo!"<<std::endl;
    resize(width,height);
    label::update(position.x(),position.y(),position.z());
    label::draw();
}

void text_label::draw(QPainter& painter)
{
    painter.setFont(QFont("System",16));
//     painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
//     painter.fillRect(0,0,width,height,Qt::green);
    painter.setPen( QColor(color.rgba[0]*255, color.rgba[1]*255, color.rgba[2]*255, color.rgba[3]*255 ) );
    painter.drawText(QRect(2,4,width, height), Qt::AlignTop, QString(text.data()));
}

} } } } // namespace snark { namespace graphics { namespace qt3d { namespace gl {
    
