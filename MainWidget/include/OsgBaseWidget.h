#pragma once

#include <osgQOpenGL/osgQOpenGLWidget>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

class OsgBaseWidget: public osgQOpenGLWidget
{
public:
	OsgBaseWidget(QWidget *paraent = nullptr);
	~OsgBaseWidget();

public slots:
	void initOsg();

private:

};
