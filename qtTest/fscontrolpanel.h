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
    
private slots:
    void on_fetchFrameButton_clicked();
    void on_hideFrameButton_clicked();

    void on_laserOnButton_clicked();
    void on_laserOffButton_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_stepLeftButton_clicked();

    void on_stepRightButton_clicked();

    void on_autoResetButton_clicked();

    void on_pushButton_clicked();

private:
    Ui::FSControlPanel *ui;
};

#endif // FSCONTROLPANEL_H
