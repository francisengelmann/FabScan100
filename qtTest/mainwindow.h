#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "staticHeaders.h"

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
    void on_pushButton_clicked();
    void on_toggleViewButton_clicked();

private:
    Ui::MainWindow *ui;
    cv::Mat image;
};

#endif // MAINWINDOW_H
