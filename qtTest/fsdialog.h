#ifndef FSDIALOG_H
#define FSDIALOG_H

#include <QDialog>
#include <QDialogButtonBox>

namespace Ui {
    class FSDialog;
}

class FSDialog : public QDialog
{
    Q_OBJECT

public:

    explicit FSDialog(QWidget *parent = 0);
    ~FSDialog();

    void setText(QString someText);
    void setStandardButtons(QDialogButtonBox::StandardButton bts);

private slots:
    void on_buttonBox_accepted();

private:
    Ui::FSDialog *ui;

};

#endif // FSDIALOG_H
