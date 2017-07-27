// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

#include "labels.h"
#include <QPainter>
#include <QOpenGLPaintDevice>
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {
    
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

} } } // namespace snark { namespace graphics { namespace qopengl {
    
