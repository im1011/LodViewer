#pragma once

#include <string>
#include <queue>
#include <thread>

#include <QMainWindow>
#include <QLabel> 
#include <Eigen/Core>

#include <Gui/OpenGlWidget.h>
#include <FileIO/BinaryIO.h>
#include <FileIO/OctreeReader.h>

// forward declarations
class QGridLayout;

namespace gui {

///
/// Window class also takes care of bundling points into blocks for rendering
/// in a single point view.
/// Inputs to the class are assumed to be in incremental order on the time axis.
/// Template type specifies precision of pose calculations.
///
template<typename T>
class Window final : public QMainWindow {
public:
    ///
    /// Constructor with reference to an octree reader instance.
    ///
    Window(const octree_reader::OctreeReader& octree_reader);
    
    ///
    ///  Virtual destructor.
    ///
    virtual ~Window();

private:
    ///
    /// Function that actually does the key event processing.
    /// 
    bool ProcessKeyPress(const QKeyEvent* const event);

private:
    ///
    /// Overrides key press event in order to be able to move the camera arround in space.
    ///
    void keyPressEvent(QKeyEvent* const event) override final;

    ///
    /// Overrides key press event in order to be able to move the camera arround in space.
    ///
    void keyReleaseEvent(QKeyEvent* const event) override final;

    ///
    /// Intercepts events from child widgets and changes to full screen mode when we left click OpenGl widget
    ///
    bool eventFilter(QObject* const watched, QEvent* const event) override final;

private:
    const octree_reader::OctreeReader& octree_reader_;
    
    QWidget main_widget_;
    OpenGlWidget<T> opengl_widget_;
};

} // namespace gui
