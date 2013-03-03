#ifndef FSSERIAL_H
#define FSSERIAL_H

#include <QObject>
#include "qextserialport-1.2beta2/src/qextserialport.h"
#include "qextserialport-1.2beta2/src/qextserialenumerator.h"

//#include "staticHeaders.h"

class FSSerial : public QObject
{

Q_OBJECT
public:
    FSSerial();
    QString *serialPortPath;
    bool connectToSerialPort();
    void writeChar(char c);
    void writeChars(char* c);

private slots:
    void onReadyRead();
    void onDsrChanged(bool);

private:
    QextSerialPort *serialPort;

};

#endif // FSSERIAL_H
