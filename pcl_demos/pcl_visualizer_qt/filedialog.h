#ifndef FILEDIALOG_H
#define FILEDIALOG_H

#include <QFileDialog>

class FileDialog : public QDialog
{
    Q_OBJECT
public:
    explicit FileDialog(QDialog *parent = 0);

public:
    enum DialogType{Image2D,Data3D};

public:
    static int Show(DialogType type, QString &pathFile);

signals:

public slots:
};

#endif // FILEDIALOG_H
