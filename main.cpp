#include "mainwindow.h"
#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("CReSIS Flight Path Planner");
    app.setApplicationVersion("1.0");

    MainWindow window;
    window.show();

    return app.exec();
}
