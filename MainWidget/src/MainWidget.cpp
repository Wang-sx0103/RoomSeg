/*****************************************************************//**
 * \file   MainWidget.cpp
 * \brief  
 * 
 * \author AlexW
 * \date   December 2023
 *********************************************************************/
#include "MainWidget.h"

#include <iostream>
#include <memory>
#include <thread>

#include <QDir>
#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QObject>
#include <QString>
#include <QTreeView>
#include <QTreeWidgetItem>

#include <pcl/PCLPointCloud2.h>

#include <Windows.h>


MainWidget::MainWidget(QWidget* parent) : QMainWindow(parent), passTreeWidgetItem(nullptr)
{
    ui.setupUi(this);

    QObject::connect(ui.fileOpenFile, &QAction::triggered, this, &MainWidget::clickedOpenFile);
    QObject::connect(ui.toolBarOpenFile, &QAction::triggered, this, &MainWidget::clickedOpenFile);
    QObject::connect(ui.treeWidget, &QTreeWidget::itemChanged, this, &MainWidget::clickedStatusTreeWidgetItem);
    QObject::connect(ui.treeWidget, &QTreeWidget::itemPressed, this, &MainWidget::pressedClickTreeWidget);
    QObject::connect(ui.toolBarSeg, &QAction::triggered, this, &MainWidget::runSemSeg);

    this->initViewer();
}

/* solts */
/**
 * \brief get pointcloud file.
 */
void MainWidget::clickedOpenFile()
{
    QStringList lPathClouds;
    // open dialog
    QFileDialog fileDialog = QFileDialog(this, QString("Open files"), QString(""),
        QString("ASCII cloud(*.txt *.asc *.neu *.xyz *,pts *.csv);;PLY mesh(*.ply);;Point Cloud Library cloud(*.pcd);;All(*.*)"));
    fileDialog.setFileMode(QFileDialog::ExistingFiles);
    fileDialog.setViewMode(QFileDialog::Detail);
    if (fileDialog.exec() == QDialog::Accepted) lPathClouds = fileDialog.selectedFiles();

    // add non-existent files to mtreeWidget
    for (QString pathFile: lPathClouds)
    {
        if (!this->inTreeWidget(pathFile))
        {
            this->mlPathCloudUnpro.push_back(pathFile);
            QList<QString>* fileNames = new QList<QString>;
            fileNames->push_back(pathFile);
            this->mtreeWidget[this->path2ItemName(pathFile)] = fileNames;

            // add QTreeWidgetItem
            QTreeWidgetItem* item = new QTreeWidgetItem();
            item->setText(0, this->path2ItemName(pathFile));
            item->setIcon(0, QIcon(":MainWidget/ico/folder.png"));
            item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
            item->setCheckState(0, Qt::Checked);
            ui.treeWidget->addTopLevelItem(item);

            // add sub QtreeWidgetItem to parent
            QTreeWidgetItem* itemSub = new QTreeWidgetItem();
            itemSub->setText(0, this->path2CloudName(pathFile));
            itemSub->setIcon(0, QIcon(":MainWidget/ico/cloud.png"));
            itemSub->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
            itemSub->setCheckState(0, Qt::Checked);
            item->addChild(itemSub);
            ui.treeWidget->update();

            // show cloud
            this->updateShowCloud(pathFile, eShowStatus::Add);
        }
    }
}

/**
 * \brief detection of changes in tree widget item.
 * 
 * \param item the selected QTreeWidgetItem
 * \param column
 */
void MainWidget::clickedStatusTreeWidgetItem(QTreeWidgetItem* item, int column)
{
    if (item->parent() == nullptr)
    {
        int index = ui.treeWidget->indexOfTopLevelItem(item);
        int numChildNode = ui.treeWidget->topLevelItem(index)->childCount();
        for (int indexSub = 0; indexSub < numChildNode; ++indexSub)
        {
            QString name = this->mtreeWidget.getSubNodeName(item->text(column), indexSub);
            if (item->checkState(column) == Qt::Checked && 
                item->child(indexSub)->checkState(column) == Qt::Checked)
            {
                this->updateShowCloud(name, eShowStatus::Show);
            }
            else
            {
                this->updateShowCloud(name, eShowStatus::Hide);
            }
        }
    }
    else
    {
        int index = ui.treeWidget->indexOfTopLevelItem(item->parent());
        int indexSub = ui.treeWidget->topLevelItem(index)->indexOfChild(item);
        QString name = this->mtreeWidget.getSubNodeName(item->parent()->text(column), indexSub);
        if (item->checkState(column) == Qt::Checked && item->parent()->checkState(column) == Qt::Checked)
        {
            this->updateShowCloud(name, eShowStatus::Show);
        }
        else
        {
            this->updateShowCloud(name, eShowStatus::Hide);
        }
    }
}

/**
 * \brief right button detedtion.
 * 
 * \param item
 * \param 
 */
void MainWidget::pressedClickTreeWidget(QTreeWidgetItem* item, int)
{
    if (qApp->mouseButtons() == Qt::RightButton)
    {
        QSharedPointer<QAction> actionInfo = QSharedPointer<QAction>(new QAction("Information", ui.treeWidget));
        QSharedPointer<QAction> actionDelete = QSharedPointer<QAction>(new QAction("Delete", ui.treeWidget));
        QSharedPointer<QMenu> menuItem = QSharedPointer<QMenu>(new QMenu(ui.treeWidget));
        menuItem->addAction(actionInfo.get());
        menuItem->addAction(actionDelete.get());
        this->passTreeWidgetItem = item;
        QObject::connect(actionInfo.get(), &QAction::triggered, this, &MainWidget::showboxCloudInfo);
        QObject::connect(actionDelete.get(), &QAction::triggered, this, &MainWidget::deleteCloud);
        menuItem->exec(QCursor::pos());
    }
}

/**
 * \brief delete TreeWidget item.
 * 
 */
void MainWidget::deleteCloud()
{
    QString nameParent;
    if (this->passTreeWidgetItem->parent() == nullptr)
    {
        int index = ui.treeWidget->indexOfTopLevelItem(this->passTreeWidgetItem);
        nameParent = this->passTreeWidgetItem->text(0);
        std::cout << nameParent.toStdString() << std::endl;
        int numChildNode = ui.treeWidget->topLevelItem(index)->childCount();
        while (numChildNode > 0)
        {
            QString removedName = this->mtreeWidget.removeSubNode(nameParent, numChildNode-1);
            delete ui.treeWidget->topLevelItem(index)->child(numChildNode - 1);
            numChildNode = ui.treeWidget->topLevelItem(index)->childCount();
            this->updateShowCloud(removedName, eShowStatus::Remove);
        }
        this->mtreeWidget.remove(nameParent);
        delete ui.treeWidget->topLevelItem(index);
    }
    else
    {
        nameParent = this->passTreeWidgetItem->parent()->text(0);
        int index = ui.treeWidget->indexOfTopLevelItem(this->passTreeWidgetItem->parent());
        int indexSub = ui.treeWidget->topLevelItem(index)->indexOfChild(this->passTreeWidgetItem);
        QString removedName = this->mtreeWidget.removeSubNode(nameParent, indexSub);
        this->updateShowCloud(removedName, eShowStatus::Remove);
        delete ui.treeWidget->topLevelItem(index)->child(indexSub);
    }
    ui.treeWidget->update();
}

/**
 * \brief show PointCloud Information.
 * 
 */
void MainWidget::showboxCloudInfo()
{
    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setWindowIcon(QIcon(":/MainWidget/ico/MainWidget.ico"));
    //msgBox.setIconPixmap(QPixmap(":/MainWidget/ico/MainWidget.ico"));
    msgBox.setWindowTitle("Information");
    msgBox.setText("Hello");
    msgBox.exec();
}

/**
 * \brief start Semantic segmentation.
 * 
 */
void MainWidget::runSemSeg()
{
    if (this->mlPathCloudUnpro.size() == 0) return;
    /*if (this->mbUnproCloud || this->mPathCloudFiles.size() == 0) return;*/
    // progress bar
    this->prgBarSemSeg = QSharedPointer<QProgressBar>(new QProgressBar(this));
    ui.statusBar->addPermanentWidget(this->prgBarSemSeg.get(), 1);
    this->prgBarSemSeg->setTextVisible(true);
    this->prgBarSemSeg->setOrientation(Qt::Horizontal);
    this->prgBarSemSeg->setValue(0);
    this->prgBarSemSeg->setMinimum(0);
    this->prgBarSemSeg->setMaximum(100);

    this->mDirCurrent = QDir::currentPath();

    // process pointcloud
    // 1 start Room Segmentation with other thread
    std::thread threadRoomSeg(&MainWidget::startSeg, this);
    threadRoomSeg.join();
    // we must wait for the threadRoomSeg for a while
    Sleep(500);
    // 2 grab the data of threadRoomSeg about processing status
    if (WaitNamedPipe(L"\\\\.\\pipe\\RoomSeg", NMPWAIT_WAIT_FOREVER) == FALSE) return;
    std::cout << "Open the pipe!" << std::endl;
    // 2.1 read | write file of pipe
    HANDLE hPipe = CreateFile(L"\\\\.\\pipe\\RoomSeg",
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if ((__int64)hPipe == -1) return;
    // 2.2 checkout status 
    const int BUFSIZE = 10;
    BOOL statusPipeFile = false;
    DWORD len = 0;
    char buffer[BUFSIZE] = { 0 };
    //std::string recvData = "";
    //QString pathCurrent = QDir::currentPath();
    //std::cout << pathCurrent.toStdString() << std::endl;
    while (true)
    {
        statusPipeFile = ReadFile(hPipe, buffer, BUFSIZE * sizeof(char), &len, NULL);
        if (!statusPipeFile) break;

        //memcpy(buffer2, buffer, len);
        std::cout << "data: " << buffer << std::endl;
        this->prgBarSemSeg->setValue(atof(buffer));
        //this->updateTreeWidget(QString("hello"));
        //recvData.append(buffer2);
    }
    //std::cout << "recv data:" << std::endl << recvData.c_str() << std::endl << std::endl;
    FlushFileBuffers(hPipe);
    DisconnectNamedPipe(hPipe);
    CloseHandle(hPipe);
    int temp = 0;
    for (QString projectCloud: this->mtreeWidget.keys())
    {
        this->mtreeWidget[projectCloud]->push_back(projectCloud);
        //this->mmStatusCloudShow[projectCloud] = true;
        // add sub QTreeWidgetItem
        QTreeWidgetItem* itemSub = new QTreeWidgetItem();
        itemSub->setText(0, this->path2CloudName(projectCloud));
        itemSub->setIcon(0, QIcon(":MainWidget/ico/cloud.png"));
        itemSub->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsEnabled);
        itemSub->setCheckState(0, Qt::Checked);
        ui.treeWidget->topLevelItem(temp)->addChild(itemSub);
        //this->updateShowCloud(projectCloud, eShowStatus::Add)
        ++temp;
    }

    // end
    this->mlPathCloudUnpro.clear();
    std::cout << "Unprocess cloud num: " << this->mlPathCloudUnpro.size() << std::endl;
}


/* function */
/**
 * \brief Initialize the renderer and windows.
 * 
 */
void MainWidget::initViewer(void)
{
    /* 1 初始化PCL画师 */
    // 初始化VTK渲染器
    this->renderer = vtkSmartPointer<vtkRenderer>::New();
    // 初始化VTK渲染窗口
    this->renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    // 添加渲染器至窗口
    this->renderWindow->AddRenderer(renderer);
    // PCL画师初始化
    this->viewCloud.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));

    /* 2 信息传递 */
    // 将PCL渲染器传递到VTK窗口中
    ui.qvtkWidget->setRenderWindow(this->viewCloud->getRenderWindow());
    // 将VTK交互信息传递至PCL画师
    this->viewCloud->setupInteractor(ui.qvtkWidget->interactor(), ui.qvtkWidget->renderWindow());
}

// check if some file has in treewidget 
bool MainWidget::inTreeWidget(QString& curPath)
{
    for (QString projectCloud: this->mtreeWidget.keys())
    {
        if (curPath.compare(projectCloud) == 0) return true;
    }
    return false;
}

/**
 * \brief Update PointCloud.
 * 
 * \param pathCloud
 * \param status
 */
void MainWidget::updateShowCloud(const QString& pathCloud, const eShowStatus status)
{
    if (status == eShowStatus::Add)
    {
        QFileInfo infoFile(pathCloud);
        QString fileType = infoFile.suffix();
        if (fileType == "pcd")
        {
            pcl::PCDReader reader;
            pcl::PCLPointCloud2::Ptr cloud = std::make_shared<pcl::PCLPointCloud2>();
            reader.readHeader(pathCloud.toStdString(), *cloud);
            if (this->getFieldType(cloud->fields) == std::string("xyzrgb")||
                this->getFieldType(cloud->fields) == std::string("rgbxyz"))
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB =
                    std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                reader.read(pathCloud.toStdString(), *cloudXYZRGB);

                Cloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud(pathCloud.toStdString());
                cloud.setCloudPtr(cloudXYZRGB);
                this->mmCloud[pathCloud] = cloud;
                // add PointCloud to PCL
                this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
                // update camera
                this->viewCloud->resetCamera();
                // refresh window
                this->viewCloud->getRenderWindow()->Render();

            }
            else if (this->getFieldType(cloud->fields) == std::string("xyzrgba") ||
                     this->getFieldType(cloud->fields) == std::string("xyzrgbalabel"))
            {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA =
                    std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
                reader.read(pathCloud.toStdString(), *cloudXYZRGBA);
                Cloud<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloud(pathCloud.toStdString());
                cloud.setCloudPtr(cloudXYZRGBA);
                this->mmCloud[pathCloud] = cloud;
                this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
                this->viewCloud->resetCamera();
                this->viewCloud->getRenderWindow()->Render();
            }
            else if (this->getFieldType(cloud->fields) == std::string("xyz"))
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ =
                    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                reader.read(pathCloud.toStdString(), *cloudXYZ);
                Cloud<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud(pathCloud.toStdString());
                cloud.setCloudPtr(cloudXYZ);
                this->mmCloud[pathCloud] = cloud;
                this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
                this->viewCloud->resetCamera();
                this->viewCloud->getRenderWindow()->Render();
            }
            //else if (this->getFieldType(cloud->fields) == std::string("xyzintensity"))
            //{
            //    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI =
            //        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            //    reader.read(pathCloud.toStdString(), *cloudXYZI);

            //    Cloud<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud(pathCloud.toStdString());
            //    cloud.setCloudPtr(cloudXYZI);
            //    //this->mlCloud.push_back(cloud);
            //    this->mmCloud[pathCloud] = cloud;

            //    // add PointCloud to PCL
            //    this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
            //    // update camera
            //    this->viewCloud->resetCamera();
            //    // refresh window
            //    this->viewCloud->getRenderWindow()->Render();
            //}
        }
        else if (fileType == "ply")
        {
            pcl::PLYReader reader;
        }
    }
    else if (status == eShowStatus::Show)
    {
        if (this->mmCloud[pathCloud].type() == typeid(Cloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>))
        {
            Cloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud =
                std::any_cast<const Cloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>&>(this->mmCloud[pathCloud]);
            this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
            this->viewCloud->resetCamera();
            this->viewCloud->getRenderWindow()->Render();
            //ui.qvtkWidget->repaint();
            //ui.qvtkWidget->update();
        }
        else if (this->mmCloud[pathCloud].type() == typeid(Cloud<pcl::PointCloud<pcl::PointXYZ>::Ptr>))
        {
            Cloud<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud =
                std::any_cast<const Cloud<pcl::PointCloud<pcl::PointXYZ>::Ptr>&>(this->mmCloud[pathCloud]);
            this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
            this->viewCloud->resetCamera();
            this->viewCloud->getRenderWindow()->Render();
        }
        else if (this->mmCloud[pathCloud].type() == typeid(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr))
        {
            Cloud<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloud =
                std::any_cast<const Cloud<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>&>(this->mmCloud[pathCloud]);
            this->viewCloud->addPointCloud(cloud.getCloudPtr(), pathCloud.toStdString());
            this->viewCloud->resetCamera();
            this->viewCloud->getRenderWindow()->Render();
        }
    }
    else if (status == eShowStatus::Hide)
    {
        // remove PointCloud from PCL
        this->viewCloud->removePointCloud(pathCloud.toStdString());
        this->viewCloud->resetCamera();
        this->viewCloud->getRenderWindow()->Render();
    }
    else
    {
        this->viewCloud->removePointCloud(pathCloud.toStdString());
        this->viewCloud->resetCamera();
        this->viewCloud->getRenderWindow()->Render();

        // delete Ptr from container

    }
}

/**
 * \brief path to name of point cloud item.
 * 
 * \param path
 * \return 
 */
QString MainWidget::path2ItemName(const QString& path)
{
    QStringList tmp = path.split("/");
    QString itemName = tmp[tmp.size() - 1] + " (" + tmp[0];
    for (int i = 1; i < tmp.size() - 1; ++i) itemName += "/" + tmp[i];
    return itemName += ")";
}

/**
 * \brief .
 * 
 * \param pathCloud
 * \return 
 */
QString MainWidget::path2CloudName(const QString pathCloud)
{
    QStringList listPathCloud = pathCloud.split("/");
    return listPathCloud[listPathCloud.size() - 1].split(".")[0];
}

/**
 * \brief start semantic segmentation by shell.
 * 
 */
void MainWidget::startSeg()
{
    // join QStringList to QString
    QString pathAll;
    for (int i = 0; i < this->mlPathCloudUnpro.size(); ++i)
    {
        pathAll += this->mlPathCloudUnpro[i] + " ";
    }
    std::cout << "All path in one: " << pathAll.toStdString() << std::endl;
    /*QString pathRoomSegExe = this->mDirCurrent+"\\RoomSeg\\Main.exe";
    const wchar_t* pathExe = TEXT("");
    ShellExecute(NULL, L"open", pathExe, NULL, NULL, SW_HIDE);*/
}
/**
 * \brief Release memory in TreeWidget.
 * 
 */
void MainWidget::clearTreeWidget()
{
    int numParentNode = ui.treeWidget->topLevelItemCount();
    while (numParentNode > 0)
    {
        int numChildNode = ui.treeWidget->topLevelItem(numParentNode - 1)->childCount();
        while (numChildNode > 0)
        {
            delete ui.treeWidget->topLevelItem(numParentNode - 1)->child(numChildNode - 1);
            numChildNode = ui.treeWidget->topLevelItem(numParentNode - 1)->childCount();
        }
        delete ui.treeWidget->topLevelItem(numParentNode - 1);
        numParentNode = ui.treeWidget->topLevelItemCount();
    }
}

/**
 * \brief .
 * 
 * \param field
 * \return 
 */
const std::string MainWidget::getFieldType(const std::vector<pcl::PCLPointField>& field)
{
    std::string temp;
    for (int i = 0; i < field.size(); ++i)
        if (!(field[i].name == std::string("_"))) temp += field[i].name;
    return temp;
}

MainWidget::~MainWidget()
{
    this->clearTreeWidget();
}
