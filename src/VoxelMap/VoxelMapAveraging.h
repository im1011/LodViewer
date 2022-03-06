#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <VoxelMap/VoxelMapAbstract.h>
#include <Geometry/Point.h>

namespace voxel_map {

///
/// Every voxel becomes the weighted average of its samples.
/// Dynamic stucture that handles point additions and deletrions.
/// Hashmap based storing of the voxelgrid based on the VoxelMapAbstract class
///
template<typename T>
class VoxelMapAveraging {
protected:
	///
	/// Running weighted average of samples inserted.
	///
	class VoxelAvg {
	public:
		///
		/// Constructor. All values zero init.
		///
		VoxelAvg();

		///
		/// Constructor with given values.
		///
		VoxelAvg(
			const Eigen::Matrix<T, 3, 1>& xyz, 
			const Eigen::Matrix<T, 3, 1>& rgb, 
			const T n
			);

		///
		/// Inserts a xyz and weight sample.
		/// Second parameter voxel_center not used for this VoxelMapAbstract implementation.
		///
		bool Insert(
			const VoxelAvg& sample,
			const Eigen::Matrix<T, 3, 1>&
			);

		///
		/// Merge another voxel's samples into this one.
		///
		bool Insert(const VoxelAvg& voxel_data);

		///
		/// Subtracts another voxel from this one.
		/// Retuns true if voxel is zeroed by the subtraction.
		///
		bool Subtract(const VoxelAvg& voxel_data);

		///
		/// Holds current weighted average xyz, rgb and sum of weights n.
		///
		Eigen::Matrix<T, 3, 1> xyz_;
		Eigen::Matrix<T, 3, 1> rgb_;
		T n_;
		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

public:
	///
	/// Constructor.
	/// When using this class make sure that 8*hash_range^3 fits into a 64 bit signed integer.
	///
	VoxelMapAveraging(
		const T voxel_size,
		const int64_t hash_range = 100000 // how many map elements can fit into the hashtable in either direction +x/-x/+y/-y/+z/-z
		);

	///
	/// There exists derived classes from this class.
	///
	virtual ~VoxelMapAveraging() { };

	///
	/// Add a bunch of points and optionally weights.
	/// If no weights vector specified, all weights are assumed to be one.
	///
	bool AddSamples(
		const std::vector<geometry::Point<T>>& points,
		const std::vector<T>& weights = {}
		);

	///
	/// Remove a bunch of points and optionally weights.
	/// If no weights vector specified, all weights are assumed to be one.
	/// If weight of a voxel drops to zero that voxel is removed from the hashmap.
	///
	void RemoveSamples(
		const std::vector<geometry::Point<T>>& points,
		const std::vector<T>& weights = {}
		);

	///
	/// Extracts all averaged points contained in the map as <xyz,rgb> pairs.
	/// Optional parameter to specify minimum weight for extracted points.
	///
	std::array<std::unique_ptr<std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>>, 2> ExtractAllPoints(
		const T min_weight = 0.0
		) const;

	///
	/// Returns the weight of the voxel in which the input point is located.
	///
	T GetWeight(const Eigen::Matrix<T, 3, 1>& point) const;

	///
	/// Returns the weight of the voxel in which the input point is located.
	///
	T GetWeight(const geometry::Point<T>& point) const;

private:
	///
	/// OpenMP parallelized generation of submaps, one for every thread
	///
	static void GenerateSubMaps(
		const std::vector<geometry::Point<T>>& points,
		const std::vector<T>& weights,
		const T voxel_size,
		const int64_t hash_range,
		std::vector<VoxelMapAbstract<T, VoxelAvg>>* const voxel_map_instances
		);

	///
	/// Merge multiple submaps into a single one
	///
	static void MergeSubMaps(
		const std::vector<VoxelMapAbstract<T, VoxelAvg>>& voxel_map_instances,
		VoxelMapAbstract<T, VoxelAvg>* const merge_to_map_instance
		);

protected:
	///
	/// Map accessor.
	///
	const VoxelMapAbstract<T, VoxelAvg>& VoxMap() const;

private:
	std::unique_ptr<VoxelMapAbstract<T, VoxelAvg>> voxel_map_;
	const T voxel_size_;
	const int64_t hash_range_;
};

} // namespace voxel_map
