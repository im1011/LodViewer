#pragma once

#include <string>
#include <vector>
#include <array>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Geometry/Point.h>

#include "BinaryIO.h"

namespace ply_io {

template <typename T>
class PlyIO {
public:
	//  TODO make a default empty vec instead
	// methods using the geometry::Point<T> struct
	static bool WritePly(
		const std::string& filename,
		const std::vector<geometry::Point<T>>& point_structs,
		const std::vector<std::array<size_t, 3>>* const triangles = nullptr
		);

	static bool ReadPly(
		const std::string & filename,
		std::vector<geometry::Point<T>>* const output_points,
		std::vector<std::array<size_t, 3>>* const output_triangles = nullptr
		);


	// lower level accessors

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const normals = nullptr,
		const std::vector<std::array<uint8_t, 3> >* const colors = nullptr,
		const std::vector<std::array<size_t, 3> >* const triangles = nullptr,
		const std::vector<T>* const intensities = nullptr
		);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const normals = nullptr,
		const std::vector<std::array<uint8_t, 3> >* const colors = nullptr,
		const std::vector<std::array<size_t, 3> >* const triangles = nullptr,
		const std::vector<T>* const intensities = nullptr
		);

	static bool WritePly(
			const std::string& filename,
			const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
			const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals
			);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<std::array<uint8_t, 3> >& colors
		);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals,
		const std::vector<std::array<uint8_t, 3> >& colors
		);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<std::array<size_t, 3> >& triangles
		);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<std::array<uint8_t, 3> >& colors,
		const std::vector<std::array<size_t, 3> >& triangles
		);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals,
		const std::vector<std::array<size_t, 3> >& triangles
		);

	static bool WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals,
		const std::vector<std::array<uint8_t, 3> >& colors,
		const std::vector<std::array<size_t, 3> >& triangles
		);

	static bool ReadPly(
		const std::string & filename,
		std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const output_points = nullptr,
		std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const output_normals = nullptr,
		std::vector<std::array<uint8_t, 3> >* const output_colors = nullptr,
		std::vector<std::array<size_t, 3> >* const output_triangles = nullptr,
		std::vector<T>* const output_intensities = nullptr
		);
};

} // namespace ply_io
