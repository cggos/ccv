#include "pclviewer.h"
#include "ui_pclviewer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "filedialog.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  action_open_file = new QAction(tr("Open Cloud..."),this);
  connect(action_open_file, &QAction::triggered, this, &PCLViewer::openCloudFile);

  action_save_file = new QAction(tr("Save Cloud..."),this);
  connect(action_save_file, &QAction::triggered, this, &PCLViewer::saveCloudFile);

  menu_file = menuBar()->addMenu(tr("Files"));
  menu_file->addAction(action_open_file);
  menu_file->addAction(action_save_file);

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  cloud->width = 200;
  cloud->height = 20;
  cloud->points.resize (cloud->width * cloud->height);

  // The default color
  red   = 128;
  green = 128;
  blue  = 128;

  // Fill the cloud with some points
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}

void
PCLViewer::randomButtonPressed ()
{
  printf ("Random button was pressed\n");

  // Set the new color
  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
  }

  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::RGBsliderReleased ()
{
  // Set the new color
  for (size_t i = 0; i < cloud->size (); i++)
  {
    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }
  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::openCloudFile ()
{
    QString strFilePath="";
    if(0 == FileDialog::Show(FileDialog::Data3D,strFilePath))
    {
        pcl::io::loadPCDFile<pcl::PointXYZRGBA>(strFilePath.toStdString(), *cloud);
        viewer->updatePointCloud (cloud, "cloud");
        ui->qvtkWidget->update ();
    }
}

void
PCLViewer::saveCloudFile ()
{
    QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "", tr ("Point cloud data (*.pcd *.ply)"));

    if (filename.isEmpty ())
        return;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::savePCDFileASCII (filename.toStdString (), *cloud);
    else if (filename.endsWith (".ply", Qt::CaseInsensitive))
        return_status = pcl::io::savePLYFileASCII (filename.toStdString (), *cloud);
    else
    {
        filename.append(".ply");
        return_status = pcl::io::savePLYFileASCII (filename.toStdString (), *cloud);
    }

    if (return_status != 0)
    {
        PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }

    PCL_INFO("Save Cloud to %s\n", filename.toStdString ().c_str ());
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
