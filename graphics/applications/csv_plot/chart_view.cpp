#include "chart_view.h"
#include <QPointF>
#include <QRectF>

namespace snark { namespace graphics { namespace plotting {
            
chart_view::chart_view( QChart* chart, QWidget* parent ) :
    QChartView( chart, parent ),
    panning_( false )
{
    setRubberBand( QChartView::RectangleRubberBand );  // rubber band: click and drag box thing
}
            
chart_view::chart_view( QWidget* parent ) :
    QChartView( parent ),
    panning_( false )
{
    setRubberBand( QChartView::RectangleRubberBand );  // rubber band: click and drag box thing
}

void chart_view::mousePressEvent( QMouseEvent* event )
{
    if ( event->button() == Qt::MiddleButton ) 
    { 
        panning_ = true;
        last_.setX(event->x());
        last_.setY(event->y());
        chart()->setAnimationOptions( QChart::NoAnimation );
    }
    QChartView::mousePressEvent( event );
}

void chart_view::mouseMoveEvent( QMouseEvent* event )
{
    if ( panning_ )
    {
        int dx = last_.x() - event->x();
        int dy = event->y() - last_.y();
        chart()->scroll( dx, dy );
        last_.setX(event->x());
        last_.setY(event->y());
    }
    QChartView::mouseMoveEvent( event );
}

void chart_view::mouseReleaseEvent( QMouseEvent* event )
{
    if ( event->button() == Qt::MiddleButton ) 
    { 
        panning_ = false; 
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
    if ( true )  // currently can't get mouse position in local frame
    {
        if      ( y > 0 ) { chart()->zoom( 1.1 );               }
        else if ( y < 0 ) { chart()->zoom( 0.9 );               }
        else              { QGraphicsView::wheelEvent( event ); }
        QPointF global_pos = event->globalPos();
        QPoint local_pos  = QChartView::mapFromGlobal( global_pos.toPoint() ); //QPoint local_pos  = chart_view().mapFromGlobal( global_pos.toPoint() );
        std::cerr << "csv-plot: mouse event occured at global position: (" << global_pos.x() << ", " << global_pos.y() << ")\n"
                  << "local position: (" << local_pos.x() << ", " << local_pos.y() << ")" << std::endl;
    }
    else
    {
        plot_area.moveCenter( event->globalPos() - chart()->scenePos() );
        if ( y > 0 )
        {
            plot_area.setTop( plot_area.top() + 10 );
            plot_area.setBottom( plot_area.bottom() - 10 );
            plot_area.setLeft( plot_area.left() + 10 );
            plot_area.setRight( plot_area.right() - 10 );
        }
        else if ( y < 0 )
        {
            plot_area.setTop( plot_area.top() - 10 );
            plot_area.setBottom( plot_area.bottom() + 10 );
            plot_area.setLeft( plot_area.left() - 10 );
            plot_area.setRight( plot_area.right() + 10 );
        }
        else
        {
            QGraphicsView::wheelEvent( event );
        }
        chart()->zoomIn( plot_area );
        std::cerr << "csv-plot: zooming in to the rectangle centred at " << plot_area.center().x() << ", " << plot_area.center().y() << std::endl;
        std::cerr << "csv-plot: mouse event occured at " << event->globalPos().x() << ", " << event->globalPos().y() << std::endl;
        std::cerr << "csv-plot: scenePos: " << chart()->scenePos().x() << ", " << chart()->scenePos().y() << std::endl;
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
