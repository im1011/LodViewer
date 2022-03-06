#pragma once

#include <memory>

#include <Gui/Views/ViewBase.h>
#include <Gui/Views/ShaderWrapper.h>

namespace gui {

class PointCloudView final : public ViewBase {
public:
	///
	/// Virtual destructor.
	///
	virtual ~PointCloudView();

	///
	/// Implements drawing. 
	///
	virtual void Draw(
		const Eigen::Matrix<float, 4, 4>& projection,
		const Eigen::Matrix<float, 4, 4>& w2v_tf
		) final override;

	///
	/// Sets points for this view. Points are copied into gpu on next draw call.
	///
	void SetPoints(
		std::unique_ptr<std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>> points,
		std::unique_ptr<std::vector<std::array<uint8_t, 4>>> point_rgba
		);

	///
	/// Sets point size for this view.
	///
	void SetPointSize(const float point_size);

	///
	/// Sets the percentage of points to render from the loaded set of points.
	///
	void SetRenderPercentage(const float render_percentage);

private:
	///
	/// Initializes the view. 
	/// Intended to initialize buffers, the shader, etc.
	///
	virtual void Init() final override;


private:
	static std::unique_ptr<ShaderWrapper> shader_;
	GLuint gl_index_xyz1_;
	GLuint gl_index_rgba_;
	GLint gl_index_w2v_;
	GLint gl_index_proj_;
	GLint gl_index_point_size_;
	GLint gl_index_alpha_;
	GLuint gl_points_buffer_;
	GLuint gl_rgba_buffer_;

	GLsizei num_points_= 0;
	std::unique_ptr<std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>> next_points_;
	std::unique_ptr<std::vector<std::array<uint8_t, 4>>> next_rgba_;

	float point_size_ = 1.0f;
	float render_percentage_ = 1.0f;
};

} // namespace gui
