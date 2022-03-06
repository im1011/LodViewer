#pragma once
#include <array>
#include <vector>

#include <Eigen/Core>

namespace geometry {

// simple 3d point 
template <typename T>
struct Point {
	Eigen::Matrix<T, 4, 1> xyz_ = (Eigen::Matrix<T, 4, 1>() << 0.0, 0.0, 0.0, 1.0).finished(); // coordinates in homogenouous coordinates XYZW (W == 1 by default)
	T i_ = -1.0; // intensity
	Eigen::Matrix<T, 3, 1> n_ = (Eigen::Matrix<T, 3, 1>() << 0.0, 0.0, 0.0).finished();  // normal
	std::array<uint8_t, 4> c_ = {{0, 0, 0, 0}}; // color (rgba)

	bool HasIntensity() const {
		return i_ >= 0.0;
	}

	bool HasNormal() const {
		const T squared_norm = n_.squaredNorm();
		return (squared_norm < 1.1 && squared_norm > 0.9);
	}

	bool HasColor() const {
		return (c_[3] != 0);
	}

	static bool PointCloudValid(const std::vector<Point<T>>& points, std::string* const error_msg) {
		if(points.empty()) {
			if(error_msg != nullptr) *error_msg = "pointcloud is empty";
			return false;
		}

		const bool has_intensity = points[0].HasIntensity();
		const bool has_normal = points[0].HasNormal();
		const bool has_color = points[0].HasColor();

		bool intensities_ok = true;
		bool normals_ok = true;
		bool colors_ok = true;

		for(const Point<T>& p : points) {
			if(p.HasIntensity() != has_intensity) intensities_ok = false;
			if(p.HasNormal() != has_normal) normals_ok = false;
			if(p.HasColor() != has_color) colors_ok = false;
		}

		if(!intensities_ok || !normals_ok || !colors_ok) {
			if(!intensities_ok && error_msg != nullptr) *error_msg += "inconsistent availability of intensity";
			if(!normals_ok && error_msg != nullptr) *error_msg += "inconsistent availability of normals";
			if(!colors_ok && error_msg != nullptr) *error_msg += "inconsistent availability of colors";
			return false;
		}

		return true;
	} 

	template <typename U>
	Point<U> Cast() const {
		Point<U> casted;
		casted.xyz_ = xyz_.template cast<U>();
		casted.i_ = static_cast<U>(i_);
		casted.n_ = n_.template cast<U>();
		casted.c_ = c_;
		return casted;
	}

public:	
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace geometry
