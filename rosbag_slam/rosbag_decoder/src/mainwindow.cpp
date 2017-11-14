#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
#include <QInputDialog>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    dirmodel =new QFileSystemModel(this);
    QString sPath="/home/redwan/Tire4/NagoyaUniversity2";
    dirmodel->setRootPath(sPath);
    dirmodel->setFilter(QDir::NoDotAndDotDot| QDir::AllDirs);
    ui->treeView->setModel(dirmodel);
    ui->treeView->setRootIndex(dirmodel->index(sPath));
    ui->treeView->hideColumn(2);
    ui->treeView->hideColumn(3);
    ui->treeView->hideColumn(1);
    filemodel =new QFileSystemModel(this);
    filemodel->setFilter(QDir::NoDotAndDotDot| QDir::Files);

    filemodel->setRootPath(sPath);
    ui->listView->setModel(filemodel);
    file =new FileReader;



}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
    QString spath= dirmodel->fileInfo(index).absoluteFilePath();
    ui->listView->setRootIndex(filemodel->setRootPath(spath));
}

void MainWindow::on_listView_clicked(const QModelIndex &index)
{
        filename= filemodel->fileInfo(index).absoluteFilePath();

}

void MainWindow::rosbag_reader(bool start_process, QStringList args)
{

    if (start_process){
        bagreader =new QProcess(this);
        ROS_WARN("process started");
        qDebug()<<args;
        bagreader->start("roslaunch",args);
        bagreader->waitForStarted();

    }
    else{

             ROS_ERROR("process terminated");
             bagreader->terminate();
             bagreader->waitForFinished();
             ROS_INFO_STREAM("EXIT STATUS "<<bagreader->exitCode());

    }

}

QString MainWindow::getVelodyneParm()
{
    QFileInfo file(filename);
    QString name=file.baseName();
    QString vd_parm;

    if (name.contains("32")){
        vd_parm="32db";
    }
    else if (name.contains("64")){
        vd_parm="64e_utexas";
    }
    else if (name.contains("16")){
        vd_parm="VLP16db";
    }
    else{
        ROS_ERROR("No parameter found");
    }
    qDebug()<<name <<":=\t"<<vd_parm;
    return vd_parm;

}


void MainWindow::on_launch_clicked(bool checked)
{
    QString data=file->read_log_file(filename);
    ui->textEdit->setTextColor(QColor("red"));
    ui->textEdit->setPlainText(data);
}

void MainWindow::on_pathAddress_clicked()
{
    QInputDialog dlg;
    QFileInfo file(filename);
    dlg.setLabelText(file.baseName());
    dlg.setTextValue(filename);
    dlg.exec();
}

void MainWindow::on_pushButton_clicked()
{
    QString data=ui->textEdit->toPlainText();
    file->dataWrite(data);
    ui->textEdit->setTextColor(QColor("black"));
    ui->textEdit->setPlainText(data);

}

//process buton
void MainWindow::on_pushButton_2_clicked()
{
dir="/home/redwan/catkin_ws/src/";
bool start_process=false;
QString button_text=ui->pushButton_2->text();
if(button_text.compare("start")==0){
 ui->pushButton_2->setText("stop");start_process=true;}
else{ ui->pushButton_2->setText("start");start_process=false;};
/******Play rosbag file *******/
QStringList velodyne_args;
velodyne_args<<dir+"velodyne.launch"<<"type:="+getVelodyneParm()<<"filename:="+filename;
rosbag_reader(start_process,velodyne_args);
/******launch  *******/

}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    ROS_INFO_STREAM(arg1);
    if(arg1){
        filemodel->setNameFilterDisables(false);
        filemodel->setNameFilters(QStringList() << "*.bag");
    }
    else{
        filemodel->setNameFilterDisables(true);
        filemodel->setNameFilters(QStringList() << "*.*");

    }

}
