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

#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include "snark/graphics/qt3d/qt3d_v2/gl_widget.h"
#include "../main_window.h"
#include "stream_data_source.h"
#include "gl_window.h"

namespace snark { namespace graphics { namespace view {

gl_window::gl_window( main_window *mw )
    : main_window_(mw)
{
    data_source_ = new stream_data_source( "-" );
    gl_widget_ = new qt3d::gl_widget( data_source_ );

    x_slider_ = create_slider();
    y_slider_ = create_slider();
    z_slider_ = create_slider();

    connect( x_slider_,  &QSlider::valueChanged, gl_widget_, &qt3d::gl_widget::setXRotation );
    connect( gl_widget_, &qt3d::gl_widget::xRotationChanged,  x_slider_, &QSlider::setValue );
    connect( y_slider_,  &QSlider::valueChanged, gl_widget_, &qt3d::gl_widget::setYRotation );
    connect( gl_widget_, &qt3d::gl_widget::yRotationChanged,  y_slider_, &QSlider::setValue );
    connect( z_slider_,  &QSlider::valueChanged, gl_widget_, &qt3d::gl_widget::setZRotation );
    connect( gl_widget_, &qt3d::gl_widget::zRotationChanged,  z_slider_, &QSlider::setValue );

    QVBoxLayout *main_layout = new QVBoxLayout;
    QHBoxLayout *container = new QHBoxLayout;
    container->addWidget( gl_widget_ );
    container->addWidget( x_slider_ );
    container->addWidget( y_slider_ );
    container->addWidget( z_slider_ );

    QWidget *w = new QWidget;
    w->setLayout( container );
    main_layout->addWidget( w );

    setLayout( main_layout );

    x_slider_->setValue( 15 * 16 );
    y_slider_->setValue( 345 * 16 );
    z_slider_->setValue( 0 * 16 );
}

gl_window::~gl_window()
{
    delete gl_widget_;
    delete data_source_;
}

QSlider *gl_window::create_slider()
{
    QSlider *slider = new QSlider(Qt::Vertical);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void gl_window::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

} } } // namespace snark { namespace graphics { namespace view {
