#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QMouseEvent>

namespace gui {

class ViewBase : protected QOpenGLExtraFunctions {
public:
	///
	/// Virtual destructor.
	///
	virtual ~ViewBase() { };

	///
	/// Draws the view if it is visible. 
	/// World to Camera coordinates transform given by w2b_tf.
	/// Projection matrix given as input too.
	///
	virtual void Draw(
		const Eigen::Matrix<float, 4, 4>& projection,
		const Eigen::Matrix<float, 4, 4>& w2v_tf
		) = 0;

	///
	/// Calls the abstract init function
	///
	virtual void Initialize() final {
		this->Init();
		is_initialized_ = true;
	};

	///
	///  Checks the initialization status
	///
	bool IsInitialized() const {
		return is_initialized_;
	}

	///
	///  Sets hidden state.
	///
	void SetHidden(const bool state) {
		is_hidden_ = state;
	}

	///
	/// Checks hidden state.
	///
	bool IsHidden() const {
		return is_hidden_;
	}

    void SetAlpha(const float alpha) {
        alpha_ = alpha;
    }

    float GetAlpha() const {
        return alpha_;
    }

private:
	///
	/// Initializes the view. 
	/// Intended to initialize buffers, the shader, etc.
	///
	virtual void Init() = 0;


private:
	bool is_hidden_ = false;
	bool is_initialized_ = false;
    float alpha_ = 1.0f;
};

} // namespace gui
