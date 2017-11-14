#include "filereader.h"

FileReader::FileReader()
{

}

QString FileReader::read_log_file(QString log){
    QFile file(log);
    fileName=log;
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    while(!file.isOpen())
        file.close();
    QString data= file.readAll();
    file.close();
    return data;
}

void FileReader::dataWrite(QString data)
{

    QFile file(fileName);
    file.open(QIODevice::Append | QIODevice::Text);

    try{
        if(file.isOpen()){
            QTextStream outStream(&file);
            outStream<<data;}
        file.close();
        ROS_INFO("File Saved Successfully");
    }
    catch (...) {
        ROS_ERROR("File Could not save!!!!!!!!");
    }


}
