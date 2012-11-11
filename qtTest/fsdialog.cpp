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

void FSDialog::setText(QString someText)
{
    ui->label->setText(someText);
}

void FSDialog::setStandardButtons(QDialogButtonBox::StandardButton bts)
{
    ui->buttonBox->setStandardButtons(bts);
}

void FSDialog::on_buttonBox_accepted()
{
    this->hide();
}
