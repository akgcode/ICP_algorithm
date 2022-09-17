#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDesktopServices>
#include <QUrl>
#include<QProcess>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_RunBotton_clicked()
{
    QDesktopServices::openUrl(QUrl("file:icp.exe",QUrl::TolerantMode));

}
