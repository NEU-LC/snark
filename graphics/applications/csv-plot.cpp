#include <iostream>
#include <QApplication>
#include "./csv_plot/plot.h"

int main( int ac, char** av )
{
    std::cerr << "csv-plot: an empty placeholder, work in progress..." << std::endl;
    QApplication a( ac, av );
    snark::plot p;
    p.show();
    return a.exec();
}
