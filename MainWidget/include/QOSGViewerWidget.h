#pragma once

#include <QOpenGLWidget>
#include <osgViewer/Viewer>

class QInputEvent;

class QOSGViewerWidget : public QOpenGLWidget, public osgViewer::Viewer
{
	Q_OBJECT

public:
	QOSGViewerWidget(QWidget *parent = 0);
	~QOSGViewerWidget();

protected:
	bool event(QEvent* event);

	void setKeyboardModifiers(QInputEvent* event);
	void keyPressEvent(QKeyEvent* event);
	void keyReleaseEvent(QKeyEvent* event);
	void mousePressEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void mouseDoubleClickEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
	void resizeEvent(QResizeEvent *event);
	void moveEvent(QMoveEvent* event);
	void timerEvent(QTimerEvent *);


protected:
	virtual void paintGL();

private:
	void init3D();

	osg::ref_ptr<osg::Camera> createCamera(int x, int y, int w, int h);

private:
	osgViewer::GraphicsWindow* _gw;
};