#ifndef FSCONTROLPANEL_H
#define FSCONTROLPANEL_H

#include "staticHeaders.h"
#include <QDialog>

namespace Ui {
    class FSControlPanel;
}

class FSControlPanel : public QDialog
{
    Q_OBJECT
    
public:
    explicit FSControlPanel(QWidget *parent = 0);
    ~FSControlPanel();
    Ui::FSControlPanel *ui;
    void setLaserAngleText(double angle);

private slots:
    void on_fetchFrameButton_clicked();

    void on_laserOnButton_clicked();

    void on_laserOffButton_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_stepLeftButton_clicked();

    void on_stepRightButton_clicked();

    void on_autoResetButton_clicked();

    void on_laserEnable_stateChanged(int arg1);

    void on_laserStepLeftButton_clicked();

    void on_laserStepRightButton_clicked();

    void on_diffImage_clicked();

    void on_laserSwipeMaxEdit_returnPressed();

    void on_laserSwipeMinEdit_returnPressed();

    void on_pushButton_2_clicked();
};

#endif // FSCONTROLPANEL_H
