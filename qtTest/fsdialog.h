#ifndef FSDIALOG_H
#define FSDIALOG_H

#include <QDialog>

namespace Ui {
    class FSDialog;
}

class FSDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FSDialog(QWidget *parent = 0);
    ~FSDialog();

private:
    Ui::FSDialog *ui;
};

#endif // FSDIALOG_H
