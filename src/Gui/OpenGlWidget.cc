#include "OpenGlWidget.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <QGuiApplication>
#include <QScreen>
#include <QApplication>
#include <QDesktopWidget>

#include <FileIO/PlyIO.h>

namespace {

///
/// Create projection matrix for opengl shader.
/// 
Eigen::Matrix<float, 4, 4> CreateFrustumMatrix(
    const float fov,
    const float aspect_ratio, 
    const float near_plane, 
    const float far_plane
    ) {
    const float scale = 1.0f / std::tan(fov * 0.5f);
    Eigen::Matrix<float, 4, 4> out;
    out(0,0) = scale / aspect_ratio;
    out(1,0) = 0.0f;
    out(2,0) = 0.0f;
    out(3,0) = 0.0f;

    out(0,1) = 0.0f;
    out(1,1) = -scale;
    out(2,1) = 0.0f;
    out(3,1) = 0.0f;
    
    out(0,2) = 0.0f;
    out(1,2) = 0.0f;
    out(2,2) = -(far_plane + near_plane) / (near_plane - far_plane);
    out(3,2) = 1.0f;
    
    out(0,3) = 0.0f;
    out(1,3) = 0.0f;
    out(2,3) = (2.f * far_plane * near_plane) / (near_plane - far_plane);
    out(3,3) = 0.0f;    

    return out;
}

template <typename T>
Eigen::Matrix<T, 3, 3> YawPitchRollToMatrix(
    const T yaw,
    const T pitch,
    const T roll
    ) {
    T cph = std::cos(roll);
    T sph = std::sin(roll);

    T ct = std::cos(pitch);
    T st = std::sin(pitch);

    T cps = std::cos(yaw);
    T sps = std::sin(yaw);

    Eigen::Matrix<T, 3, 3> rot;

    rot(0, 0) = ct * cps;
    rot(0, 1) = cps * st * sph - cph * sps;
    rot(0, 2) = cph * cps * st + sph * sps;

    rot(1, 0) = ct * sps;
    rot(1, 1) = cph * cps + st * sph * sps;
    rot(1, 2) = -cps * sph + cph * st * sps;

    rot(2, 0) = -st;
    rot(2, 1) = ct * sph;
    rot(2, 2) = ct * cph;

    return rot;
}

template <typename T>
Eigen::Matrix<T, 4, 4> YawPitchRollTranslationToMatrix(
    const T yaw,
    const T pitch,
    const T roll,
    const Eigen::Matrix<T, 3, 1>& translation
    ) {
    const Eigen::Matrix<T, 3, 3> rot = YawPitchRollToMatrix(yaw, pitch, roll);

    Eigen::Matrix<T, 4, 4> transform;
    transform.template block<3,3>(0,0) = rot;
    transform.template block<3,1>(0,3) = translation;
    transform.template block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;
    return transform;
}

template <typename T>
Eigen::Matrix<T, 4, 4> InverseTransform(const Eigen::Matrix<T, 4, 4>& transform) {
    Eigen::Matrix<T, 3, 3> rotation = transform.template block<3,3>(0,0);
    Eigen::Matrix<T, 3, 1> translation = transform.template block<3,1>(0,3);
    rotation.transposeInPlace();
    translation = - rotation * translation;

    Eigen::Matrix<T, 4, 4> inv_transform;
    inv_transform.template block<3,3>(0,0) = rotation;
    inv_transform.template block<3,1>(0,3) = translation;
    inv_transform.template block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;
    return inv_transform;
}


} // namespace

namespace gui {

template <typename T>
OpenGlWidget<T>::OpenGlWidget(const octree_reader::OctreeReader& octree_reader) : octree_reader_(octree_reader) {
    prev_draw_time_ = std::chrono::high_resolution_clock::now();

    current_b2v_ = YawPitchRollTranslationToMatrix(
        static_cast<T>(0.0),
        static_cast<T>(-M_PI_2),
        static_cast<T>(M_PI_2), {
            static_cast<T>(0.0), 
            static_cast<T>(0.0), 
            static_cast<T>(0.0)
        }
    );

    gravity_axis_in_view_(0) = static_cast<T>(0.0);
    gravity_axis_in_view_(1) = static_cast<T>(1.0);
    gravity_axis_in_view_(2) = static_cast<T>(0.0);
    ref_point_ = QPoint(-1, -1);
}



template <typename T>
void OpenGlWidget<T>::InitAllViews() {
    const size_t screen_num_pixels = static_cast<size_t>(QApplication::desktop()->screenGeometry().height() * QApplication::desktop()->screenGeometry().width());
    octree_view_.reset(new OctreeView(octree_reader_, screen_num_pixels));
}

template <typename T>
bool OpenGlWidget<T>::SetForwardTranslation(const bool state) {
    if(state == translating_forward_)
        return false;
    translating_forward_ = state;
    return true;
}

template <typename T>
bool OpenGlWidget<T>::SetBackwardTranslation(const bool state) {
    if(state == translating_backward_)
        return false;
    translating_backward_ = state;
    return true;
}

template <typename T>
bool OpenGlWidget<T>::SetLeftTranslation(const bool state) {
    if(state == translating_left_)
        return false;
    translating_left_ = state;
    return true;
}

template <typename T>
bool OpenGlWidget<T>::SetRightTranslation(const bool state) {
    if(state == translating_right_)
        return false;
    translating_right_ = state;
    return true;
}

template <typename T>
bool OpenGlWidget<T>::UpDownTranslation(const float val) {
    current_b2v_(1,3) += up_down_translation_speed_ * val * GetTranslationSpeedup();
    return true;
}

template <typename T>
void OpenGlWidget<T>::IncrementTranslation(const float delta_t) {
    const float translation_step = delta_t * translation_velocity_ * GetTranslationSpeedup();

    float tz = 0.0f;
    float tx  = 0.0f;

    if(translating_forward_)
        tz -= translation_step;
    if(translating_backward_)
        tz += translation_step;
    if(translating_left_)
        tx += translation_step;
    if(translating_right_)
        tx -= translation_step;

    current_b2v_(0,3) += tx;
    current_b2v_(2,3) += tz;
}

template <typename T>
void OpenGlWidget<T>::paintGL() {
    ProcessPointSizeSelection();

    const std::chrono::time_point<std::chrono::high_resolution_clock> draw_time_now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> delta_t = draw_time_now - prev_draw_time_;
    prev_draw_time_ = draw_time_now;

    IncrementTranslation(static_cast<float>(delta_t.count()));
    UpdateRotation();

    const Eigen::Matrix<float, 4, 4> projection_matrix = GetProjectionMatrix();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if(!octree_view_->IsInitialized())
        octree_view_->Initialize();
    octree_view_->Draw(
        projection_matrix, 
        (current_b2v_).template cast<float>()
        );

    update();
}

template <typename T>
void OpenGlWidget<T>::initializeGL() {
    // Initialize OpenGL functions to resolve to the current context. If context changes it has to be called again
    initializeOpenGLFunctions();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    InitAllViews();
}

template <typename T>
void OpenGlWidget<T>::mousePressEvent(QMouseEvent* const event) {
    if(event->button() == Qt::LeftButton){
        ref_point_ = QCursor::pos();
        return;
    }

    QWidget::mousePressEvent(event);
}

template <typename T>
void OpenGlWidget<T>::mouseReleaseEvent(QMouseEvent* const event) {
    if(ref_point_.x() > 0 && ref_point_.y() > 0) {
        ref_point_ = QPoint(-1, -1);
        return;
    }

    QWidget::mouseReleaseEvent(event);
}

template <typename T>
float OpenGlWidget<T>::GetAspectRatio() const {
    float aspect_ratio = 1.0f;
    if(size().height() > 0)
        aspect_ratio = (float)size().width() / (float)size().height();

    return aspect_ratio;
}

template <typename T>
Eigen::Matrix<float, 4, 4> OpenGlWidget<T>::GetProjectionMatrix() {
    return CreateFrustumMatrix(fov_, GetAspectRatio(), 0.1f,  10000.0f);
}

template <typename T>
void OpenGlWidget<T>::UpdateRotation() {
    if(ref_point_.x() < 0 || ref_point_.y() < 0)
        return;

    QPoint cursor_loc = QCursor::pos();
    const int delta_x = cursor_loc.x() - ref_point_.x();
    const int delta_y = cursor_loc.y() - ref_point_.y();

    const Eigen::Matrix<T, 3, 3> r_lr = 
        Eigen::AngleAxis<T>(static_cast<T>(delta_x) * static_cast<T>(0.01), -gravity_axis_in_view_).toRotationMatrix();
    const Eigen::Matrix<T, 3, 3> r_ud = 
        Eigen::AngleAxis<T>(static_cast<T>(delta_y) * static_cast<T>(0.01), Eigen::Matrix<T, 3, 1>::UnitX()).toRotationMatrix();

    Eigen::Matrix<T, 4, 4> tf_combi = Eigen::Matrix<T, 4, 4>::Identity();
    tf_combi.template block<3,3>(0,0) = r_ud * r_lr;

    current_b2v_ = tf_combi * current_b2v_;
    gravity_axis_in_view_ = r_ud * r_lr * gravity_axis_in_view_;

    ref_point_ = QCursor::pos();
}

template <typename T>
float OpenGlWidget<T>::GetTranslationSpeedup() const {
    const size_t level = reinterpret_cast<OctreeView*>(octree_view_.get())->GetLowestLevel();

    switch(level) {
        case 0: return 25.0f;
        case 1: return 25.0f;
        case 2: return 25.0f;
        case 3: return 20.0f;
        case 4: return 10.0f;
        case 5: return 6.0f;
        case 6: return 4.0f;
    }
    return 1.0f;
}

template <typename T>
void OpenGlWidget<T>::SetOctreePointSize(const float point_size) {
    buffer_point_size_octree_ = point_size;
}


template <typename T>
void OpenGlWidget<T>::ProcessPointSizeSelection() {
    if(octree_view_->IsInitialized())
        reinterpret_cast<OctreeView*>(octree_view_.get())->SetPointSize(buffer_point_size_octree_);
}

template <typename T>
void OpenGlWidget<T>::resizeEvent(QResizeEvent* event) {
    QOpenGLWidget::resizeEvent(event);
}

template class OpenGlWidget<float>;
template class OpenGlWidget<double>;

} // namespace gui
