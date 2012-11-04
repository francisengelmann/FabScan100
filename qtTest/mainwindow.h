#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "staticHeaders.h"

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
    
private slots:
    void on_myButton_clicked();
    void on_convertButton_clicked();
    void on_toggleViewButton_clicked();

    void on_pingButton_clicked();

    void timerEvent(QTimerEvent *e);

    void onSelectSerialPort();
    void openPointCloud();
    void newPointCloud();

    //void onDataAvailable();
    //void onReadyRead(); //oudated
    //void onDsrChanged(bool); //oudated
    //void sendChar(char c); //oudated

private:
    QBasicTimer *hwTimer; //updates connected hw:arduino,webcam,...
    Ui::MainWindow *ui;
    cv::Mat image;
    //QString *serialPortPath; //oudated
    //QextSerialPort *serialPort; //oudated

    void setupMenu();

    //serial port functionality
    //bool connectToSerialPort(); //outdated
    void enumerateSerialPorts();
};

#endif // MAINWINDOW_H
