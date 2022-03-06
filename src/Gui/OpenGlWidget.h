#pragma once

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <list>

#include <Eigen/Core>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QMouseEvent>

#include <Gui/Views/ViewBase.h>
#include <Gui/Views/PointCloudView/PointCloudView.h>
#include <Gui/Views/OctreeView/OctreeView.h>
#include <FileIO/OctreeReader.h>

namespace gui {

///
/// Template type specifies precision of pose calculations.
///
template<typename T>
class OpenGlWidget final : public QOpenGLWidget, protected QOpenGLExtraFunctions {
public:
    ///
    /// Constructor with reference to an octree reader instance.
    ///
    OpenGlWidget(
        const octree_reader::OctreeReader& octree_reader
        );

    ///
    /// Virtual destructor since its a derived class.
    ///
    virtual ~OpenGlWidget() { };


public:   
    ///
    /// Sets translation. Returns false if the state is not toggled.
    ///
    bool SetForwardTranslation(const bool state);
    
    ///
    /// Sets translation. Returns false if the state is not toggled.
    ///
    bool SetBackwardTranslation(const bool state);
    
    ///
    /// Sets translation. Returns false if the state is not toggled.
    ///
    bool SetLeftTranslation(const bool state);
    
    ///
    /// Sets translation. Returns false if the state is not toggled.
    ///
    bool SetRightTranslation(const bool state);

    ///
    /// Translates up and down by a fixed amount.
    /// Positive values translate up.
    ///
    bool UpDownTranslation(const float val);
    
    ///
    /// Adjusts the rendered point size.
    ///
    void SetOctreePointSize(const float point_size);

private:
    ///
    /// Adjusts translation of the view matrix based on the state booleans.
    ///
    void IncrementTranslation(const float delta_t);

    ///
    /// Initializes all views and fills the pointers.
    ///
    void InitAllViews();

    ///
    /// Updates the rotation of the scene view when the widget is in full screen mode
    ///
    void UpdateRotation();

    ///
    /// Returns the aspect ratio of the current widget
    ///
    float GetAspectRatio() const;

    Eigen::Matrix<float, 4, 4> GetProjectionMatrix();

    ///
    /// Accelerate the tranlsation of the view with increasing distance to the octree points.
    ///
    float GetTranslationSpeedup() const;

    ///
    /// Processes the pending point size selection in the draw thread
    ///
    void ProcessPointSizeSelection();

private:
    ///
    /// Callback on a loop that renders the openGL content. 
    /// Override of the Qt base function.
    ///
    void paintGL() final override;

    ///
    /// Override of the Qt base function.
    ///
    void initializeGL() final override;

    ///
    /// Override of the Qt base function.
    ///
    void mousePressEvent(QMouseEvent* const event) final override;

    ///
    /// Override of the Qt base function.
    ///
    void mouseReleaseEvent(QMouseEvent* const event) final override;

    ///
    /// Override of the Qt base function.
    ///
    void resizeEvent(QResizeEvent* event) final override;

private:
    const octree_reader::OctreeReader& octree_reader_;

    // variables for handling translation of view
    bool translating_forward_ = false;
    bool translating_backward_ = false;
    bool translating_left_ = false;
    bool translating_right_ = false;

    // base translation velocities
    const float translation_velocity_ = 10.0f;
    const float up_down_translation_speed_ = 0.005f;

    std::chrono::time_point<std::chrono::high_resolution_clock> prev_draw_time_;
    QPoint ref_point_;

    // view transform and projection parameters
    Eigen::Matrix<T, 4, 4> current_b2v_;
    Eigen::Matrix<T, 3, 1> gravity_axis_in_view_;
    float fov_ = static_cast<float>(M_PI_2);

    // Owners of all views
    std::unique_ptr<ViewBase> octree_view_;
    float buffer_point_size_octree_ = 1.0f;

public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace gui
