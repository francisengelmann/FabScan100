#include "fscontrolpanel.h"
#include "ui_fscontrolpanel.h"
#include "fscontroller.h"

#include <QDebug>

FSControlPanel::FSControlPanel(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FSControlPanel)
{
    ui->setupUi(this);
    qDebug("controlpanel constr called");
}

FSControlPanel::~FSControlPanel()
{
    delete ui;
}

void FSControlPanel::on_fetchFrameButton_clicked()
{
    FSController::getInstance()->fetchFrame();
}

void FSControlPanel::on_hideFrameButton_clicked()
{
    FSController::getInstance()->hideFrame();
}

void FSControlPanel::on_laserOnButton_clicked()
{
    FSController::getInstance()->turnLaserOn();
}

void FSControlPanel::on_laserOffButton_clicked()
{
    FSController::getInstance()->turnLaserOff();
}

void FSControlPanel::on_checkBox_stateChanged(int state)
{
    if(state==2){
        FSController::getInstance()->stepper->enable();
    }else{
        FSController::getInstance()->stepper->disable();
    }
}

void FSControlPanel::on_stepLeftButton_clicked()
{
    FSController::getInstance()->stepper->setDirection(FS_DIRECTION_CW);
    QString str = ui->degreesEdit->text();
    double degrees = str.toDouble();
    FSController::getInstance()->stepper->turnNumberOfDegrees(degrees);
}

void FSControlPanel::on_stepRightButton_clicked()
{
    FSController::getInstance()->stepper->setDirection(FS_DIRECTION_CCW);
    QString str = ui->degreesEdit->text();
    double degrees = str.toDouble();
    FSController::getInstance()->stepper->turnNumberOfDegrees(degrees);
}

void FSControlPanel::on_autoResetButton_clicked()
{
    FSController::getInstance()->detectLaserLine();
}

void FSControlPanel::on_pushButton_clicked()
{
    this->hide();
}
