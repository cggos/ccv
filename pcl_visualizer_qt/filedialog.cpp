#include "filedialog.h"

FileDialog::FileDialog(QDialog *parent):QDialog(parent)
{
}

int FileDialog::Show(DialogType type,QString &pathFile)
{
    QString selfilter = "";
    switch(type)
    {
    case Image2D:
        selfilter = QObject::tr("Image Files(*.png *.jpg *.bmp);;All Files(*)");
        break;
    case Data3D:
        selfilter = QObject::tr("Point Cloud Files(*.pcd *.ply);;All Files(*)");
        break;
    }

    QFileDialog *dlgFile = new QFileDialog(NULL,"File Dialog",".",selfilter);
    dlgFile->setFilter(QDir::Files);
    dlgFile->setViewMode(QFileDialog::List);
    dlgFile->setFileMode(QFileDialog::ExistingFile);

    if(dlgFile->exec() == QDialog::Accepted)
    {
        QStringList pathList = dlgFile->selectedFiles();
        QFileInfo infoFile = QFileInfo(pathList.at(0));

        QString nameFile = infoFile.fileName();
        QString dirFile = infoFile.absolutePath();
        pathFile = infoFile.filePath();

        return 0;
    }
    else
    {
        return -1;
    }
}
