// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#ifndef SNARK_GRAPHICS_QT3D_GLWIDGET_H_
#define SNARK_GRAPHICS_QT3D_GLWIDGET_H_

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "snark/graphics/applications/view_points/reader.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

namespace snark { namespace graphics { namespace qt3d {

class gl_widget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

    public:
        gl_widget( view::Reader* reader, QWidget *parent = 0 );
        ~gl_widget();

        QSize minimumSizeHint() const Q_DECL_OVERRIDE;
        QSize sizeHint() const Q_DECL_OVERRIDE;

    public slots:
        void setXRotation( int angle );
        void setYRotation( int angle );
        void setZRotation( int angle );
        void cleanup();

    signals:
        void xRotationChanged( int angle );
        void yRotationChanged( int angle );
        void zRotationChanged( int angle );

    protected:
        void initializeGL() Q_DECL_OVERRIDE;
        void paintGL() Q_DECL_OVERRIDE;
        void resizeGL( int width, int height ) Q_DECL_OVERRIDE;
        void mousePressEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;
        void mouseMoveEvent( QMouseEvent *event ) Q_DECL_OVERRIDE;

    private:
        void setupVertexAttribs();

        int xRot_;
        int yRot_;
        int zRot_;
        QPoint last_pos_;
        view::Reader* reader_;
        QOpenGLVertexArrayObject vao_;
        QOpenGLBuffer vbo_;
        QOpenGLShaderProgram *program_;
        int projection_matrix_location_;
        int mv_matrix_location_;
        QMatrix4x4 projection_;
        QMatrix4x4 camera_;
        QMatrix4x4 world_;
};

} } } // namespace snark { namespace graphics { namespace qt3d {

#endif /*SNARK_GRAPHICS_QT3D_GLWIDGET_H_*/
