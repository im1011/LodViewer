#pragma once

#include <array>

#include <Eigen/Core>

namespace voxel_map {

template<typename T>
class KeyGenerate {
public:
	KeyGenerate(
		const T voxel_size,
		const int64_t hash_range = 100000
		) : hash_range_(hash_range),
			inverse_voxel_size_(1.0f / voxel_size) {
	}

	int64_t GetVoxelId(const int64_t i, const int64_t j, const int64_t k) const {
		return (i+hash_range_) + 2*hash_range_*(j+hash_range_) + 4*hash_range_*hash_range_*(k+hash_range_);
	}

	int64_t GetVoxelId(const Eigen::Matrix<T, 4, 1>& xyz) const {
		const std::array<int64_t, 3> ijk = GetVoxelPosition(xyz);
		const int64_t i = ijk[0];
		const int64_t j = ijk[1];
		const int64_t k = ijk[2];

		return GetVoxelId(i, j, k);
	}

private:
	std::array<int64_t, 3> GetVoxelPosition(const Eigen::Matrix<T, 4, 1>& xyz) const {
		return {
			static_cast<int64_t>(xyz(0) * inverse_voxel_size_) + (xyz(0) < static_cast<T>(0.0) ? -1 : 0),
			static_cast<int64_t>(xyz(1) * inverse_voxel_size_) + (xyz(1) < static_cast<T>(0.0) ? -1 : 0),
			static_cast<int64_t>(xyz(2) * inverse_voxel_size_) + (xyz(2) < static_cast<T>(0.0) ? -1 : 0)
		};
	}

private:
	const int64_t hash_range_;
	const T inverse_voxel_size_;
};

} // namespace voxel_map
