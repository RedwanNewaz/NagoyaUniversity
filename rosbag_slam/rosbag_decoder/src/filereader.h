#ifndef FILEREADER_H
#define FILEREADER_H
#include<QtCore>
#include <QString>
#include <ros/ros.h>

class FileReader
{
public:
    FileReader();
    QString read_log_file(QString log);
    void dataWrite(QString data);
private:
    QString fileName;
};

#endif // FILEREADER_H
