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

#include <Windows.h>


MainWidget::MainWidget(QWidget *parent): QMainWindow(parent), passTreeWidgetItem(nullptr)
{
    ui.setupUi(this);

    QObject::connect(ui.fileOpenFile, &QAction::triggered, this, &MainWidget::clickedOpenFile);
    QObject::connect(ui.toolBarOpenFile, &QAction::triggered, this, &MainWidget::clickedOpenFile);
    QObject::connect(ui.treeWidget, &QTreeWidget::itemChanged, this, &MainWidget::changedStatusTreeWidget);
    QObject::connect(ui.treeWidget, &QTreeWidget::itemPressed, this, &MainWidget::pressedClickTreeWidget);
    QObject::connect(ui.toolBarSeg, &QAction::triggered, this, &MainWidget::runSemSeg);
    
    this->initViewer();
}

/* solts*/
// load files
void MainWidget::clickedOpenFile()
{
    QStringList lPathClouds;
    // open dialog
    QFileDialog fileDialog = QFileDialog(this, QString("Open files"), QString(""),
        QString("ASCII cloud(*.txt *.asc *.neu *.xyz *,pts *.csv);;PLY mesh(*.ply);;Point Cloud Library cloud(*.pcd);;All(*.*)"));
    fileDialog.setFileMode(QFileDialog::ExistingFiles);
    fileDialog.setViewMode(QFileDialog::Detail);
    if (fileDialog.exec() == QDialog::Accepted) lPathClouds = fileDialog.selectedFiles();
    //if (this->ui.treeWidget->topLevelItemCount() > 0) this->clearTreeWidget();
    
    for (QString pathFile: lPathClouds)
    {
        // check file if in mlPathCloudFilesRef
        //if (!this->inPathCloudRef(pathFile))
        if (!this->inTreeWidget(pathFile))
        {
            //this->mlPathCloudFilesRef.push_back(pathFile);
            this->mlPathCloudUnpro.push_back(pathFile);
            this->mmStatusCloudShow[pathFile] = true;
            QList<QString> *fileName = new QList<QString>;
            //QSharedPointer<QList<QString>> fileName = QSharedPointer<QList<QString>>(new QList<QString>);
            fileName->push_back(pathFile);
            this->mtreeWidget[this->path2ItemName(pathFile)] = fileName;
        }
    }
    // refresh TreeWidget
    this->updateTreeWidget();
    // Refresh display
    this->updateShowCloud();
}

// detection of changes in tree widget item
void MainWidget::changedStatusTreeWidget(QTreeWidgetItem* item, int column)
{
    //std::cout << "My column is: " << column << std::endl;
    if (item->parent() == nullptr)
    {
        int index = ui.treeWidget->indexOfTopLevelItem(item);
        std::cout << "parent node: " << index << std::endl;
    }
    else
    {
        int index = ui.treeWidget->indexOfTopLevelItem(item->parent());
        int indexSub = ui.treeWidget->topLevelItem(index)->indexOfChild(item);
        QString name = this->mtreeWidget.getSubNodeName(item->parent()->text(0), indexSub);
        this->mmStatusCloudShow[name] = this->mmStatusCloudShow[name] == true ? false : true;
        std::cout << "child node: " << indexSub << this->mmStatusCloudShow[name] << std::endl;
    }
    // Refresh display
    this->updateShowCloud();
}

// right button detedtion
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

// delete TreeWidget item
void MainWidget::deleteCloud()
{
    // Refresh display
    std::cout << "delete cloud..." << std::endl;
    if (this->passTreeWidgetItem->parent() == nullptr)
    {
        int index = ui.treeWidget->indexOfTopLevelItem(this->passTreeWidgetItem);
        std::cout << this->passTreeWidgetItem->text(0).toStdString() << std::endl;
        //int numChildNode = ui.treeWidget->topLevelItem(index)->childCount();
        /*for (int indexSub = 0; indexSub < numChildNode; indexSub++)
        {
            QString removedName = this->mtreeWidget.removeSubNode(this->passTreeWidgetItem->text(0), indexSub);
            this->mmStatusCloudShow.remove(removedName);
        }*/
        this->mtreeWidget.remove(this->passTreeWidgetItem->text(0));
        //std::cout << "parent node: " << index << std::endl;
    }
    else
    {
        int index = ui.treeWidget->indexOfTopLevelItem(this->passTreeWidgetItem->parent());
        int indexSub = ui.treeWidget->topLevelItem(index)->indexOfChild(this->passTreeWidgetItem);
        QString removedName = this->mtreeWidget.removeSubNode(this->passTreeWidgetItem->parent()->text(0), indexSub);
        this->mmStatusCloudShow.remove(removedName);
    }
    this->updateTreeWidget();
    this->updateShowCloud();
}

// show point cloud Information
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

// start Semantic segmentation
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
    char buffer[BUFSIZE] = {0};
    //std::string recvData = "";
    QString pathCurrent = QDir::currentPath();
    std::cout << pathCurrent.toStdString() << std::endl;
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

    for (QString projectCloud: this->mtreeWidget.keys())
    {
        this->mtreeWidget[projectCloud]->push_back(projectCloud);
        this->mmStatusCloudShow[projectCloud] = true;
    }
    // end
    this->mlPathCloudUnpro.clear();
    std::cout << "Unprocess cloud num: " << this->mlPathCloudUnpro.size() << std::endl;
    this->updateTreeWidget();
    this->updateShowCloud();
}


/* function */

void MainWidget::initViewer(void)
{
    //初始化VTK的渲染器
    this->renderer = vtkSmartPointer<vtkRenderer>::New();
    this->renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    this->renderWindow->AddRenderer(renderer);
    // 初始化PCL
    this->viewCloud.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));

    //将渲染器加入到VTK窗口中
    ui.qvtkWidget->setRenderWindow(this->viewCloud->getRenderWindow());
    this->viewCloud->setupInteractor(ui.qvtkWidget->interactor(), ui.qvtkWidget->renderWindow());
}

// check if some file has in treewidget 
bool MainWidget::inTreeWidget(QString &curPath)
{
    for (QString projectCloud: this->mtreeWidget.keys())
    {
        if (curPath.compare(projectCloud) == 0) return true;
    }
    return false;
}

/**

*/
void MainWidget::updateTreeWidget(void)
{
    /*Afterwards, we will set up detailed update strategies
    to reduce unnecessary memory releases and development*/
    this->clearTreeWidget();
    ui.treeWidget->clear();
    for (QString path: this->mtreeWidget.keys())
    {
        std::cout << "parent node: " << path.toStdString() << std::endl;
        QTreeWidgetItem* item = new QTreeWidgetItem();
        item->setText(0, path);
        item->setIcon(0, QIcon(":MainWidget/ico/folder.png"));
        item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsEnabled);
        item->setCheckState(0, Qt::Checked);
        ui.treeWidget->addTopLevelItem(item);
        for (QString name: *this->mtreeWidget[path])
        {
            std::cout << "child node: " << name.toStdString() << std::endl;
            QTreeWidgetItem* itemSub = new QTreeWidgetItem();
            itemSub->setText(0, this->path2CloudName(name));
            itemSub->setIcon(0, QIcon(":MainWidget/ico/cloud.png"));
            itemSub->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsEnabled);
            itemSub->setCheckState(0, Qt::Checked);
            item->addChild(itemSub);
        }
    }
}

// show point cloud in ...
void MainWidget::updateShowCloud(void)
{
    //this->ui.openGLWidget->getOsgViewer()->setSceneData(this->viewerRoot);
    // Show point clouds with true values in mmPathCloudStatus
    std::cout << "we will show those point-clouds: " << std::endl;

    for (QString pathCloud : this->mmStatusCloudShow.keys())
    {
        std::cout
            << "point cloud name: " << pathCloud.toStdString()
            << "and its status: " << this->mmStatusCloudShow[pathCloud]
            << std::endl;
        QFileInfo infoFile(pathCloud);
        QString fileType = infoFile.suffix();
        if (fileType == "pcd")
        {

        }
    }

}

// 
//void MainWidget::addFiles(QStringList& cloudFiles)
//{
//    for (QString file: cloudFiles)
//    {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//        QFileInfo filepath(file);
//        QString fileType = filepath.suffix();
//        if (fileType == "ply")
//        {
//            if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(file.toStdString(), *pCloud) == -1) PCL_ERROR("Couldn't read file test_pcd.pcd \n");
//            //this->viewerRoot->addChild(this->cloud2Geometry(pCloud));
//        }
//        else if (fileType == "pcd")
//        {
//            pcl::PCDReader reader;
//            reader.read<pcl::PointXYZRGB>(file.toStdString(), *pCloud);
//            //this->viewerRoot->addChild(this->cloud2Geometry(pCloud));
//        }
//    }
//}

// path to name of point cloud item
QString MainWidget::path2ItemName(const QString& path)
{
    QStringList tmp = path.split("/");
    QString itemName = tmp[tmp.size() - 1]+" (" + tmp[0];
    for (int i = 1; i < tmp.size()-1; ++i) itemName += "/" + tmp[i];
    return itemName += ")";
}

QString MainWidget::path2CloudName(const QString pathCloud)
{
    QStringList listPathCloud = pathCloud.split("/");
    return listPathCloud[listPathCloud.size() - 1].split(".")[0];
}

// start semantic segmentation by shell
void MainWidget::startSeg()
{
    // join QStringList to QString
    QString pathAll;
    for (int i = 0; i < this->mlPathCloudUnpro.size(); ++i)
    {
        pathAll += this->mlPathCloudUnpro[i] + " ";
    }
    std::cout << "All path in one: " << pathAll.toStdString() << std::endl;
    ShellExecute(NULL, L"open", L"D:\\Develop\\Little\\dist\\Pipe.exe", NULL, NULL, SW_HIDE);
}

void MainWidget::clearTreeWidget()
{
    int numParentNode = ui.treeWidget->topLevelItemCount();
    //std::cout << "num Parent node: " << numParentNode << std::endl;
    while (numParentNode>0)
    {
        int numChildNode = ui.treeWidget->topLevelItem(numParentNode-1)->childCount();
        //std::cout << "num child node: " << numChildNode << std::endl;
        while (numChildNode>0)
        {
            // have a bug!!!
            delete ui.treeWidget->topLevelItem(numParentNode-1)->child(numChildNode-1);
            numChildNode = ui.treeWidget->topLevelItem(numParentNode - 1)->childCount();
            //std::cout << "num child node: " << numChildNode << std::endl;
        }
        delete ui.treeWidget->topLevelItem(numParentNode-1);
        numParentNode = ui.treeWidget->topLevelItemCount();
        //std::cout << "num Parent node: " << numParentNode << std::endl;
    }
    //QTreeWidgetItemIterator itTopNode(ui.treeWidget);
    //while (*itTopNode)
    //{
    //    //std::cout << (*itTopNode)->child(0)->text(0).toStdString() << std::endl;
    //    QTreeWidgetItemIterator itChildNode((*itTopNode)->child(0));
    //    while (*itChildNode)
    //    {
    //        std::cout << (*itChildNode)->text(0).toStdString() << std::endl;
    //        delete *itChildNode;
    //    }
    //    std::cout << (*itTopNode)->text(0).toStdString() << std::endl;
    //    delete *itTopNode;
    //    //++itTopNode;
    //}
}

MainWidget::~MainWidget()
{ 
    this->clearTreeWidget();
    delete this->passTreeWidgetItem;
}
