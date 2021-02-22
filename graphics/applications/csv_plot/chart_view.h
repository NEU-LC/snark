// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <iostream>  // todo: try without this
#include <QWheelEvent>
#include <QMouseEvent>
#include <QPoint>
#include <QPointF>
#include <QRectF>
#include <QtCharts/QChartView>
#include <QtWidgets/QRubberBand>

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

namespace snark { namespace graphics { namespace plotting {

class chart_view: public QChartView
{
public:
    chart_view( QChart* chart, QWidget* parent = 0 );

protected:
    void keyPressEvent( QKeyEvent* event );
    void wheelEvent( QWheelEvent* event );
    void mousePressEvent( QMouseEvent* event );
    void mouseMoveEvent( QMouseEvent* event );
    void mouseReleaseEvent( QMouseEvent* event );
    
private:
    bool panning;
    int last_x;
    int last_y;
};

} } } // namespace snark { namespace graphics { namespace plotting {
