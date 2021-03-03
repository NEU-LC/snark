#include "chart_view.h"

namespace snark { namespace graphics { namespace plotting {
            
chart_view::chart_view( QChart* chart, QWidget* parent ) :
    QChartView( chart, parent ),
    panning( false )
{
    setRubberBand( QChartView::RectangleRubberBand );  // rubber band: click and drag box thing
}

void chart_view::mousePressEvent( QMouseEvent* event )
{
    if ( event->button() == Qt::MiddleButton ) 
    { 
        panning = true;
        last_x = event->x();
        last_y = event->y();
        chart()->setAnimationOptions( QChart::NoAnimation );
    }
    QChartView::mousePressEvent( event );
}

void chart_view::mouseMoveEvent( QMouseEvent* event )
{
    if ( panning )
    {
        int dx = last_x - event->x();
        int dy = event->y() - last_y;
        chart()->scroll( dx, dy );
        last_x = event->x();
        last_y = event->y();
    }
    QChartView::mouseMoveEvent( event );
}

void chart_view::mouseReleaseEvent( QMouseEvent* event )
{
    if ( event->button() == Qt::MiddleButton ) 
    { 
        panning = false; 
        chart()->setAnimationOptions( QChart::SeriesAnimations );
    }
    QChartView::mouseReleaseEvent( event );
}

void chart_view::wheelEvent( QWheelEvent* event )
{
    QPoint angle_delta = event->angleDelta();
    QRectF plot_area = chart()->plotArea();
    int y = angle_delta.y();  // +ve if scrolling in
    // if ( chart()->chartType() == QChart::ChartTypePolar )  // polar charts don't support rectangle zooming
    if ( true )  // polar charts don't support rectangle zooming
    {
        if( y == 0 ) { QGraphicsView::wheelEvent( event ); }
        else { chart()->zoom( y > 0 ? 1.1 : 0.5 )  }
    }
    else  // currently can't get mouse position in local frame
    {
        plot_area.moveCenter( event->globalPos() - chart()->scenePos() );
        if( y == 0 )
        {
            QGraphicsView::wheelEvent( event );
        }
        else
        {
            int sign = y > 0 ? 1 : -1;
            plot_area.setTop( plot_area.top() + 10 * sign );
            plot_area.setBottom( plot_area.bottom() - 10 * sign );
            plot_area.setLeft( plot_area.left() + 10 * sign );
            plot_area.setRight( plot_area.right() - 10 * sign );
        }
        chart()->zoomIn( plot_area );
        std::cerr << "Zooming in to the rectangle centred at " << plot_area.center().x() << ", " << plot_area.center().y() << std::endl;
        std::cerr << "Mouse event occured at " << event->globalPos().x() << ", " << event->globalPos().y() << std::endl;
        std::cerr << "scenePos: " << chart()->scenePos().x() << ", " << chart()->scenePos().y() << std::endl;
    }
}

void chart_view::keyPressEvent( QKeyEvent* event )
{
    switch ( event->key() ) {
    case Qt::Key_Plus:
        chart()->zoomIn();
        break;
    case Qt::Key_Minus:
        chart()->zoomOut();
        break;
    case Qt::Key_Left:
        chart()->scroll(-50, 0);
        break;
    case Qt::Key_Right:
        chart()->scroll(50, 0);
        break;
    case Qt::Key_Up:
        chart()->scroll(0, 50);
        break;
    case Qt::Key_Down:
        chart()->scroll(0, -50);
        break;
    case Qt::Key_R:
        chart()->zoomReset();
        break;
    default:
        QGraphicsView::keyPressEvent(event);
        break;
    }
}

} } } // namespace snark { namespace graphics { namespace plotting {

