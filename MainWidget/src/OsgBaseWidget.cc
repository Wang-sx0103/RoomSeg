#include "OsgBaseWidget.h"

OsgBaseWidget::OsgBaseWidget(QWidget *parent): osgQOpenGLWidget(parent)
{
	OsgBaseWidget* OSGWidget;
	this->setFocusPolicy(Qt::StrongFocus);
	connect(this, &OsgBaseWidget::initialized, this, &OsgBaseWidget::initOsg);
}

void OsgBaseWidget::initOsg()
{
	this->getOsgViewer()->setCameraManipulator(new osgGA::TrackballManipulator);
	this->getOsgViewer()->addEventHandler(new osgViewer::StatsHandler);
	this->getOsgViewer()->addEventHandler(new osgViewer::HelpHandler);
	this->getOsgViewer()->addEventHandler(new osgViewer::ScreenCaptureHandler);
	this->getOsgViewer()->addEventHandler(new osgGA::StateSetManipulator(getOsgViewer()->getCamera()->getOrCreateStateSet()));
}

OsgBaseWidget::~OsgBaseWidget(){}
