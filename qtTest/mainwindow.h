#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "staticHeaders.h"
#include "fsdialog.h"
#include "fscontrolpanel.h"


QT_BEGIN_NAMESPACE
class QBasicTimer;
class QGLShaderProgram;
QT_END_NAMESPACE

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void showDialog(QString dialogText);
    void redraw();
    FSControlPanel* controlPanel;


private slots:
    void on_convertButton_clicked();
    void on_toggleViewButton_clicked();
    void on_scanButton_clicked();

    void timerEvent(QTimerEvent *e);

    void onSelectSerialPort();
    void onSelectWebCam();
    void openPointCloud();
    void savePointCloud();
    void newPointCloud();
    void showControlPanel();

private:
    QBasicTimer *hwTimer; //updates connected hw:arduino,webcam,...
    Ui::MainWindow *ui;

    FSDialog* dialog;

    void setupMenu();
    void enumerateSerialPorts();
    void enumerateWebCams();
};

#endif // MAINWINDOW_H
