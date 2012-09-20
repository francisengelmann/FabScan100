#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_myButton_clicked()
{
    //image= cv::imread("cube.png");
    //cv::namedWindow("Original Image");
    //cv::imshow("Original Image", image);
    ui->label->setText("Press a button to close...");
    qDebug("Hello World!");


    // Open the video file
          cv::VideoCapture capture(-1);
          // check if video successfully opened
          if (!capture.isOpened()) return;
          // Get the frame rate
          double rate= capture.get(CV_CAP_PROP_FPS);
          bool stop(false);
          cv::Mat frame; // current video frame
          cv::namedWindow("Extracted Frame");
          // Delay between each frame in ms
          // corresponds to video frame rate
          int delay= 1000/rate;
          // for all frames in video
          while (!stop) {
             // read next frame if any
             if (!capture.read(frame))
                break;
             cv::imshow("Extracted Frame",frame);
             // introduce a delay
             // or press key to stop
             if (cv::waitKey(1)>=0)
            stop= true;
    }
          // Close the video file.
          // Not required since called by destructor
          capture.release();
          cvDestroyWindow("Extracted Frame");
          ui->label->setText("Idling...");

}
