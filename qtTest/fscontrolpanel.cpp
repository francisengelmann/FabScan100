#include "fscontrolpanel.h"
#include "ui_fscontrolpanel.h"
#include "fscontroller.h"
#include "fslaser.h"
#include "fsturntable.h"
#include "fsserial.h"

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
    FSController::getInstance()->laser->turnOn();
}

void FSControlPanel::on_laserOffButton_clicked()
{
    FSController::getInstance()->laser->turnOff();
}

void FSControlPanel::on_checkBox_stateChanged(int state)
{
    if(state==2){
        FSController::getInstance()->turntable->enable();
    }else{
        FSController::getInstance()->turntable->disable();
    }
}

void FSControlPanel::on_stepLeftButton_clicked()
{
    FSController::getInstance()->turntable->setDirection(FS_DIRECTION_CW);
    QString str = ui->degreesEdit->text();
    double degrees = str.toDouble();
    FSController::getInstance()->turntable->turnNumberOfDegrees(degrees);
}

void FSControlPanel::on_stepRightButton_clicked()
{
    FSController::getInstance()->turntable->setDirection(FS_DIRECTION_CCW);
    QString str = ui->degreesEdit->text();
    double degrees = str.toDouble();
    FSController::getInstance()->turntable->turnNumberOfDegrees(degrees);
}

void FSControlPanel::on_autoResetButton_clicked()
{
    FSController::getInstance()->detectLaserLine();
    cv::Mat shot = FSController::getInstance()->webcam->getFrame();
    cv::resize( shot,shot,cv::Size(1280,960) );
    shot = FSController::getInstance()->vision->drawLaserLineToFrame(shot);
    cv::resize(shot,shot,cv::Size(800,600));
    cv::imshow("Laser Frame",shot);
    cv::waitKey(0);
    cvDestroyWindow("Laser Frame");
    this->raise();
    this->focusWidget();
}

void FSControlPanel::on_pushButton_clicked()
{
    this->hide();
}

void FSControlPanel::on_binaryImage_clicked()
{
    cv::Mat shot = FSController::getInstance()->subLaser();
    cv::resize(shot,shot,cv::Size(800,600));
    cv::imshow("Laser Frame",shot);
    cv::waitKey(0);
    cvDestroyWindow("Laser Frame");
}
