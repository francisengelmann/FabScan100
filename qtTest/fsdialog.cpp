#include "fsdialog.h"
#include "ui_fsdialog.h"

FSDialog::FSDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FSDialog)
{
    ui->setupUi(this);
}

FSDialog::~FSDialog()
{
    delete ui;
}
