/*****************************************************************//**
 * \file   MainWidget.h
 * \brief  
 * 
 * \author AlexW
 * \date   December 2023
 *********************************************************************/
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

class MainWidget: public QMainWindow
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
    const QString path2ItemName(const QString&);
    const QString path2CloudName(const QString&);
    const QString path2PathSeg(const QString&);
    const QString path2PathTran(const QString&);
    void startSeg();
    bool inTreeWidget(QString&);
    void updateShowCloud(const QString&, const eShowStatus);
    //void updateTreeWidget();
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
    // Cloud Pointer
    QMap<QString, std::any> mmCloud;

    // organizational treeWidget structure
    TreeCatalog mtreeWidget;

    // Global variables is used to pass parameters
    QTreeWidgetItem* passTreeWidgetItem;

    // show Semantic segmentation progress
    QSharedPointer<QProgressBar> prgBarSemSeg;

    // current dir
    QString mDirCurrent;

    // from VTK renderer for PCL
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;

    // show point cloud by PCL
    pcl::visualization::PCLVisualizer::Ptr viewCloud;
};
