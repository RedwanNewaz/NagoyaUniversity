#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QFileSystemModel>
#include <QDebug>
#include "filereader.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_checkBox_stateChanged(int arg1);

private slots:
    void on_pushButton_2_clicked();

private slots:
    void on_pushButton_clicked();

private slots:
    void on_pathAddress_clicked();

private slots:
    void on_launch_clicked(bool checked);

private slots:


    void on_treeView_clicked(const QModelIndex &index);

    void on_listView_clicked(const QModelIndex &index);
protected:
    void rosbag_reader(bool,QStringList args);
    QString getVelodyneParm();
private:
    Ui::MainWindow *ui;
    QFileSystemModel *dirmodel;
    QFileSystemModel *filemodel;
    FileReader *file;
    QString filename,dir;
    QProcess *velodyne, *bagreader,*hector;

};

#endif // MAINWINDOW_H
