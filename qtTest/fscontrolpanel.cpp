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
        FSController::getInstance()->turnStepperOn();
    }else{
        FSController::getInstance()->turnStepperOff();
    }
}

void FSControlPanel::on_stepLeftButton_clicked()
{
    FSController::getInstance()->setDirectionCCW();
    FSController::getInstance()->performStep();
}

void FSControlPanel::on_stepRightButton_clicked()
{
    FSController::getInstance()->setDirectionCW();
    FSController::getInstance()->performStep();}

void FSControlPanel::on_autoResetButton_clicked()
{
    FSController::getInstance()->detectLaserLine();
}

void FSControlPanel::on_pushButton_clicked()
{
    this->hide();
}
