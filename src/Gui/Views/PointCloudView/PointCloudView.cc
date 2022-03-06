#include "PointCloudView.h"

#include <filesystem>
#include <array>

#include <QApplication>

namespace {

std::filesystem::path GetLocalDirectory() {
    std::filesystem::path file_path(__FILE__);
    return file_path.parent_path();
}


std::string VertShaderPath() {
    std::filesystem::path shader_rel_path("PointCloudView.vs");
    std::filesystem::path file_path(QCoreApplication::applicationDirPath().toStdString());
    file_path /= "Views";
    file_path /= "PointCloudView";
    file_path /= shader_rel_path;
    if (std::filesystem::exists(file_path))
        return file_path.string();

    file_path = GetLocalDirectory();
    file_path = file_path / shader_rel_path;
    
    if (std::filesystem::exists(file_path))
        return file_path.string();
    
    file_path = std::filesystem::path(QCoreApplication::applicationDirPath().toStdString());
    file_path /= "PointCloudView.vs";
    
    return file_path.string();
}

std::string FragShaderPath() {
    std::filesystem::path shader_rel_path("PointCloudView.fs");
    std::filesystem::path file_path(QCoreApplication::applicationDirPath().toStdString());
    file_path /= "Views";
    file_path /= "PointCloudView";
    file_path /= shader_rel_path;
    if (std::filesystem::exists(file_path))
        return file_path.string();

    file_path = GetLocalDirectory();
    file_path = file_path / shader_rel_path;
    
    if (std::filesystem::exists(file_path))
        return file_path.string();
    
    file_path = std::filesystem::path(QCoreApplication::applicationDirPath().toStdString());
    file_path /= "PointCloudView.fs";

    return file_path.string();
}

} // namespace

namespace gui {

std::unique_ptr<ShaderWrapper> PointCloudView::shader_(nullptr);

PointCloudView::~PointCloudView() {
    if (!IsInitialized())
        return;

    glDeleteBuffers(1, &gl_points_buffer_);
    glDeleteBuffers(1, &gl_rgba_buffer_);
}

void PointCloudView::Init() {
    initializeOpenGLFunctions();
    if (shader_ == nullptr)
        shader_.reset(new ShaderWrapper(VertShaderPath(), FragShaderPath()));

    gl_index_xyz1_ = static_cast<GLuint>(glGetAttribLocation(shader_->Id(), "xyz1_"));
    gl_index_rgba_ = static_cast<GLuint>(glGetAttribLocation(shader_->Id(), "rgba_"));
    gl_index_w2v_ = glGetUniformLocation(shader_->Id(), "w2v_");
    gl_index_proj_ = glGetUniformLocation(shader_->Id(), "proj_");
    gl_index_point_size_ = glGetUniformLocation(shader_->Id(), "point_size_");
    gl_index_alpha_ = glGetUniformLocation(shader_->Id(), "alpha_");

    shader_->Use();
    glGenBuffers(1, &gl_points_buffer_);
    glGenBuffers(1, &gl_rgba_buffer_);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void PointCloudView::Draw(
    const Eigen::Matrix<float, 4, 4>& projection,
    const Eigen::Matrix<float, 4, 4>& w2v_tf
    ) {
    if(IsHidden())
        return;
    if (!IsInitialized())
        return;
    if(num_points_ == 0 && next_points_ == nullptr)
        return;

    shader_->Use();

    glUniformMatrix4fv(gl_index_w2v_, 1, GL_FALSE, w2v_tf.data());
    glUniformMatrix4fv(gl_index_proj_, 1, GL_FALSE, projection.data());
    glUniform1f(gl_index_point_size_, point_size_);
    glUniform1f(gl_index_alpha_, this->GetAlpha());

    if(next_points_ != nullptr && next_rgba_!= nullptr
        && next_points_->size() == next_rgba_->size()) {
        glBindBuffer(GL_ARRAY_BUFFER, gl_points_buffer_);
        glBufferData(
            GL_ARRAY_BUFFER, 
            static_cast<GLsizeiptr>(next_points_->size() * 4 * sizeof(float)), 
            &(next_points_->at(0)(0)), 
            GL_STREAM_DRAW
            );
        glBindBuffer(GL_ARRAY_BUFFER, gl_rgba_buffer_);
        glBufferData(
            GL_ARRAY_BUFFER, 
            static_cast<GLsizeiptr>(next_rgba_->size() * 4 * sizeof(uint8_t)), 
            &(next_rgba_->at(0)), 
            GL_STREAM_DRAW
            );

        num_points_ = static_cast<GLsizei>(next_points_->size());
        next_points_.reset(nullptr);
        next_rgba_.reset(nullptr);
    }

    glEnableVertexAttribArray(gl_index_xyz1_);
    glBindBuffer(GL_ARRAY_BUFFER, gl_points_buffer_);
    glVertexAttribPointer(gl_index_xyz1_, 4, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(gl_index_rgba_);
    glBindBuffer(GL_ARRAY_BUFFER, gl_rgba_buffer_);
    glVertexAttribPointer(gl_index_rgba_, 4, GL_UNSIGNED_BYTE, GL_FALSE, 0, 0);
    
    if(render_percentage_ == 1.0f) {
        glDrawArrays(GL_POINTS, 0, num_points_);
    } else {
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(render_percentage_ * static_cast<float>(num_points_) + 1.01f));
    }

    glDisableVertexAttribArray(gl_index_xyz1_);
    glDisableVertexAttribArray(gl_index_rgba_);
}

void PointCloudView::SetPoints(
    std::unique_ptr<std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>> points,
    std::unique_ptr<std::vector<std::array<uint8_t, 4>>> point_rgba
    ) {
    if(points->size() != point_rgba->size())
        return;
    if(points->size() == 0)
        return;
    
    next_points_ = std::move(points);
    next_rgba_ = std::move(point_rgba);
}

void PointCloudView::SetPointSize(const float point_size) {
    point_size_ = point_size;
}

void PointCloudView::SetRenderPercentage(const float render_percentage) {
    render_percentage_ = render_percentage;
}

} // namespace gui
