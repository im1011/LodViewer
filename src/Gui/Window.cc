#include "Window.h"

#include <iomanip>
#include <filesystem>

#include <QGridLayout>
#include <QKeyEvent>
#include <QApplication>
#include <QDesktopWidget>


namespace gui {
	
template <typename T>
Window<T>::Window(const octree_reader::OctreeReader& octree_reader) : octree_reader_(octree_reader),
        opengl_widget_(octree_reader) {
            
    opengl_widget_.installEventFilter(this);

    QLayout *ini_layout = main_widget_.layout();
    if (ini_layout != nullptr)
        delete ini_layout;

    showMaximized();
    QGridLayout* layout = new QGridLayout();
    opengl_widget_.setMinimumSize(200,200);
    layout->addWidget(&opengl_widget_, 0, 0);

    layout->setColumnStretch(0, 1);
    main_widget_.setLayout(layout);

    setCentralWidget(&main_widget_);
}

template <typename T>
Window<T>::~Window() {
}

template <typename T>
void Window<T>::keyPressEvent(QKeyEvent* const event) {
    if (!ProcessKeyPress(event))
        QMainWindow::keyPressEvent(event);
}

template <typename T>
void Window<T>::keyReleaseEvent(QKeyEvent* const event) {
    if (!ProcessKeyPress(event))
        QMainWindow::keyPressEvent(event);
}

template <typename T>
bool Window<T>::ProcessKeyPress(const QKeyEvent* const event) {
    if(event->isAutoRepeat())
        return false;

    const int key = event->key();

    if (key == Qt::Key_W){
        return opengl_widget_.SetForwardTranslation(event->type() == QEvent::KeyPress);
    } else if (key == Qt::Key_S) {
        return opengl_widget_.SetBackwardTranslation(event->type() == QEvent::KeyPress);
    } else if (key == Qt::Key_A) {
        return opengl_widget_.SetLeftTranslation(event->type() == QEvent::KeyPress);
    } else if (key == Qt::Key_D) {
        return opengl_widget_.SetRightTranslation(event->type() == QEvent::KeyPress);
    } else {
        return false;
    }
}

template <typename T>
bool Window<T>::eventFilter(QObject* const obj, QEvent* const event)
{
    // Intercept left click on the OpenGL widget and enable full screen mode on double click 
    if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
        const QKeyEvent* const key_event = static_cast<QKeyEvent*>(event);
        ProcessKeyPress(key_event);
        return true;
    } else if (event->type() == QEvent::Wheel) {
        const QWheelEvent* const wheel_event = static_cast<QWheelEvent*>(event);
        opengl_widget_.UpDownTranslation(static_cast<float>(wheel_event->angleDelta().y()));
    }

    return QObject::eventFilter(obj, event);
}

template class Window<float>;
template class Window<double>;

} // namespace gui
