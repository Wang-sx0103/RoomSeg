#pragma once

#include "ui_MainWidget.h"
#include "TreeCatalog.h"
#include "Cloud.hpp"

#include <memory>
#include <any>

#include <QList>
#include <QMap>
#include <QProgressBar>
#include <QSharedPointer>
#include <QString>
#include <QStringList>
#include <QTreeWidgetItemIterator>
#include <QtWidgets/QMainWindow>

#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class MainWidget : public QMainWindow
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

private slots:
    void clickedOpenFile();
    void clickedStatusTreeWidgetItem(QTreeWidgetItem*, int);
    void pressedClickTreeWidget(QTreeWidgetItem*, int);
    void runSemSeg();
    void deleteCloud();
    void showboxCloudInfo();

private:
    // 
    enum eShowStatus { Add, Show, Hide, Remove };

    void initViewer(void);
    QString path2ItemName(const QString&);
    //QString itemName2path(const QString&);
    QString path2CloudName(const QString);
    void startSeg();
    //bool inPathCloudRef(QString&);
    bool inTreeWidget(QString&);
    void updateShowCloud();
    void updateShowCloud(const QString&, const eShowStatus);
    void updateTreeWidget();
    void clearTreeWidget();

    // get point cloud field info
    const std::string getFieldType(const std::vector<pcl::PCLPointField>&);

private:
    // ui object
    Ui::Windows ui;
 
    /* They are used for displaying point clouds
    and organizing treeWidgets */
    // Path of unprocessed point clouds
    QStringList mlPathCloudUnpro;
    // Status of displayed point cloud
    QMap<QString, bool> mmStatusCloudShow;
    // Cloud Pointer
    //QList<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mlpCloud;
    QList<std::any> mlCloud;

    // organizational treeWidget structure
    TreeCatalog mtreeWidget;

    // Global variables is used to pass parameters
    QTreeWidgetItem* passTreeWidgetItem;

    // show Semantic segmentation progress
    QSharedPointer<QProgressBar> prgBarSemSeg;

    // renderer
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    // show point cloud by PCL
    pcl::visualization::PCLVisualizer::Ptr viewCloud;
};
