
#include <QMenu>
#include <osg/LightModel>
#include <osgViewer/Renderer>
#include <osg/ValueObject>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#pragma execution_character_set("utf-8")
#include "qosgviewerwidget.h"

#include <QInputEvent>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgGA/MultiTouchTrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <QApplication>
#include <QMessageBox>


QOSGViewerWidget::QOSGViewerWidget(QWidget *parent)
	: QOpenGLWidget(parent)
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setSamples(16);
	setFormat(format);

	init3D();
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
}

QOSGViewerWidget::~QOSGViewerWidget()
{
}

bool QOSGViewerWidget::event(QEvent *event)
{
	switch (event->type()) {
	case QEvent::TouchBegin:
	case QEvent::TouchEnd:
	case QEvent::TouchUpdate: {
		QList<QTouchEvent::TouchPoint> touchPoints = static_cast<QTouchEvent *>(event)->touchPoints();
		unsigned int id = 0;
		unsigned int tapCount = touchPoints.size();

		osg::ref_ptr<osgGA::GUIEventAdapter> osgEvent(NULL);
		osgGA::GUIEventAdapter::TouchPhase phase = osgGA::GUIEventAdapter::TOUCH_UNKNOWN;
		foreach(const QTouchEvent::TouchPoint& touchPoint, touchPoints) {
			if (!osgEvent) {
				if (event->type() == QEvent::TouchBegin) {
					phase = osgGA::GUIEventAdapter::TOUCH_BEGAN;
					osgEvent = _gw->getEventQueue()->touchBegan(id, osgGA::GUIEventAdapter::TOUCH_BEGAN, touchPoint.pos().x(), touchPoint.pos().y());
				}
				else if (event->type() == QEvent::TouchEnd) {
					phase = osgGA::GUIEventAdapter::TOUCH_ENDED;
					osgEvent = _gw->getEventQueue()->touchEnded(id, osgGA::GUIEventAdapter::TOUCH_ENDED, touchPoint.pos().x(), touchPoint.pos().y(), tapCount);
				}
				else if (event->type() == QEvent::TouchUpdate) {
					phase = osgGA::GUIEventAdapter::TOUCH_MOVED;
					osgEvent = _gw->getEventQueue()->touchMoved(id, osgGA::GUIEventAdapter::TOUCH_MOVED, touchPoint.pos().x(), touchPoint.pos().y());
				}
			}
			else {
				osgEvent->addTouchPoint(id, osgGA::GUIEventAdapter::TOUCH_ENDED, touchPoint.pos().x(), touchPoint.pos().y());
				osgEvent->addTouchPoint(id, phase, touchPoint.pos().x(), touchPoint.pos().y());
			}
			id++;
		}
		break;
	}
	default:
		break;
	}
	return QOpenGLWidget::event(event);
}

void QOSGViewerWidget::setKeyboardModifiers(QInputEvent *event)
{
	int modkey = event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier);
	unsigned int mask = 0;
	if (modkey & Qt::ShiftModifier) {
		mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
	}
	if (modkey & Qt::ControlModifier) {
		mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
	}
	if (modkey & Qt::AltModifier) {
		mask |= osgGA::GUIEventAdapter::MODKEY_ALT;
	}

	_gw->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
	update();
}

void QOSGViewerWidget::keyPressEvent(QKeyEvent *event)
{
	QMessageBox::information(NULL, "Title", "Content", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
	setKeyboardModifiers(event);
	_gw->getEventQueue()->keyPress(event->key());
	QOpenGLWidget::keyPressEvent(event);
	update();
}

void QOSGViewerWidget::keyReleaseEvent(QKeyEvent *event)
{
	setKeyboardModifiers(event);
	_gw->getEventQueue()->keyRelease(event->key());
	QOpenGLWidget::keyReleaseEvent(event);
	update();
}

void QOSGViewerWidget::mousePressEvent(QMouseEvent *event)
{
	int button = 0;
	switch (event->button()) {
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	case Qt::NoButton: button = 0; break;
	default: button = 0; break;
	}
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
	update();
}

void QOSGViewerWidget::mouseReleaseEvent(QMouseEvent *event)
{
	int button = 0;
	switch (event->button()) {
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	case Qt::NoButton: button = 0; break;
	default: button = 0; break;
	}
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);

	QOpenGLWidget::mouseReleaseEvent(event);
	update();
}

void QOSGViewerWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	int button = 0;
	switch (event->button()) {
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	case Qt::NoButton: button = 0; break;
	default: button = 0; break;
	}
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseDoubleButtonPress(event->x(), event->y(), button);

	QOpenGLWidget::mouseDoubleClickEvent(event);
	update();
}

void QOSGViewerWidget::mouseMoveEvent(QMouseEvent *event)
{
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseMotion(event->x(), event->y());
	QOpenGLWidget::mouseMoveEvent(event);
	update();
}

void QOSGViewerWidget::wheelEvent(QWheelEvent *event)
{
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseScroll(
		event->orientation() == Qt::Vertical ?
		(event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN) :
		(event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_LEFT : osgGA::GUIEventAdapter::SCROLL_RIGHT));
	QOpenGLWidget::wheelEvent(event);
	update();
}

void QOSGViewerWidget::resizeEvent(QResizeEvent *event)
{
	const QSize& size = event->size();
	_gw->resized(x(), y(), size.width(), size.height());
	_gw->getEventQueue()->windowResize(x(), y(), size.width(), size.height());
	_gw->requestRedraw();

	QOpenGLWidget::resizeEvent(event);
}

void QOSGViewerWidget::moveEvent(QMoveEvent *event)
{
	const QPoint& pos = event->pos();
	_gw->resized(pos.x(), pos.y(), width(), height());
	_gw->getEventQueue()->windowResize(pos.x(), pos.y(), width(), height());

	QOpenGLWidget::moveEvent(event);
}

void QOSGViewerWidget::timerEvent(QTimerEvent *)
{
	update();
}

void QOSGViewerWidget::paintGL()
{
	if (isVisibleTo(QApplication::activeWindow())) {
		frame();
	}
}

void QOSGViewerWidget::init3D()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->windowDecoration = false;
	traits->x = 0;
	traits->y = 0;
	traits->width = width();
	traits->height = height();
	traits->doubleBuffer = true;
	traits->sharedContext = 0;

	_gw = new osgViewer::GraphicsWindowEmbedded(traits);

	realize();

	


	auto camera = getCamera();
	camera->setGraphicsContext(_gw);
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	this->setCamera(camera);//这两句话的先后顺序
	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(osgDB::readNodeFile("cow.osg"));//注意：这两句话的先后顺序 先添加模型在添加相机
	this->setSceneData(root);
	
	this->setThreadingModel(osgViewer::Viewer::SingleThreaded);
	this->setCameraManipulator(new osgGA::TrackballManipulator);
	this->addEventHandler(new osgViewer::WindowSizeHandler());

	startTimer(20);
}

osg::ref_ptr<osg::Camera> QOSGViewerWidget::createCamera(int x, int y, int w, int h)
{
	_gw = new osgViewer::GraphicsWindowEmbedded(x, y, w, h);

	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(_gw);
	camera->setViewport(new osg::Viewport(0, 0, w, h));
	camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camera->setProjectionMatrixAsPerspective(
		30.0f, double(w) / double(h), 1.0f, 10000.0f);
	camera->setClearColor(osg::Vec4(0.3, 0.3, 0.6, 0.1));

	return camera.release();
}
