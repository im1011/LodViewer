#include "VoxelMapAveraging.h"

#include <omp.h>

namespace voxel_map {

template <typename T>
VoxelMapAveraging<T>::VoxelAvg::VoxelAvg() : xyz_(0.0, 0.0, 0.0), rgb_(0.0, 0.0, 0.0), n_(0.0) { }

template <typename T>
VoxelMapAveraging<T>::VoxelAvg::VoxelAvg(
	const Eigen::Matrix<T, 3, 1>& xyz, 
	const Eigen::Matrix<T, 3, 1>& rgb, 
	const T n
	) : xyz_(xyz), rgb_(rgb), n_(n) { 
}

template <typename T>
bool VoxelMapAveraging<T>::VoxelAvg::Insert(
	const VoxelAvg& sample,
	const Eigen::Matrix<T, 3, 1>& // voxel_center not used
	) {
	return Insert(sample);
}

template <typename T>
bool VoxelMapAveraging<T>::VoxelAvg::Insert(const VoxelAvg& voxel_data) {
	n_ += voxel_data.n_;
	xyz_ += voxel_data.n_ / n_ * (voxel_data.xyz_ - xyz_);
	rgb_ += voxel_data.n_ / n_ * (voxel_data.rgb_ - rgb_);
	return true;
}

template <typename T>
bool VoxelMapAveraging<T>::VoxelAvg::Subtract(const VoxelAvg& voxel_data) {
	n_ -= voxel_data.n_;
	if(n_ < 0.0001)
		return true;
	xyz_ -= voxel_data.n_ / n_ * (voxel_data.xyz_ - xyz_);
	rgb_ -= voxel_data.n_ / n_ * (voxel_data.rgb_ - rgb_);
	return false;
}

template <typename T>
VoxelMapAveraging<T>::VoxelMapAveraging(
	const T voxel_size,
	const int64_t hash_range // how many map elements can fit into the hashtable in either direction +x/-x/+y/-y/+z/-z
	) : voxel_size_(voxel_size),
		hash_range_(hash_range) {
	voxel_map_.reset(new VoxelMapAbstract<T, VoxelAvg>(voxel_size, hash_range));
}

template <typename T>
void VoxelMapAveraging<T>::GenerateSubMaps(
	const std::vector<geometry::Point<T>>& points,
	const std::vector<T>& weights,
	const T voxel_size,
	const int64_t hash_range,
	std::vector<VoxelMapAbstract<T, VoxelAvg>>* const voxel_map_instances
	) {
	// create voxel grids so that the different cores can work on their own without sync needs
	voxel_map_instances->clear();
	size_t num_threads = static_cast<size_t>(omp_get_max_threads());
	for(size_t i=0; i < num_threads; ++i)
		voxel_map_instances->push_back(VoxelMapAbstract<T, VoxelAvg>(voxel_size, hash_range));

	bool insertion_loop_corrupt = false;
	
	#pragma omp parallel for schedule(dynamic)
	for(size_t i=0; i < points.size(); ++i) {
		if(insertion_loop_corrupt)
			continue;

		const VoxelAvg xyzn_sample(
			points[i].xyz_.template block<3,1>(0,0), 
			{
				static_cast<T>(points[i].c_[0]),
				static_cast<T>(points[i].c_[1]),
				static_cast<T>(points[i].c_[2])
			},
			(weights.empty() ? static_cast<T>(1.0) : weights[i])
			);
		const bool ret = voxel_map_instances->at(static_cast<size_t>(omp_get_thread_num())).Insert(xyzn_sample);
		if(!ret)
			insertion_loop_corrupt = true;
	}
}

template <typename T>
void VoxelMapAveraging<T>::MergeSubMaps(
	const std::vector<VoxelMapAbstract<T, VoxelAvg>>& voxel_map_instances,
	VoxelMapAbstract<T, VoxelAvg>* const merge_to_map_instance
	) {
	// merge the individual voxel map instances
	// TODO now its a simple O(n) loop; implement this in O(log(n))
	for(size_t i=0; i < voxel_map_instances.size(); ++i)
		merge_to_map_instance->MergeMap(voxel_map_instances[i]);
}

template <typename T>
bool VoxelMapAveraging<T>::AddSamples(
	const std::vector<geometry::Point<T>>& points,
	const std::vector<T>& weights
	) {
	// generate submaps and merge the individual voxel map instances into the class's global one
	std::vector<VoxelMapAbstract<T, VoxelAvg>> voxel_map_instances;
	GenerateSubMaps(points, weights, voxel_size_, hash_range_, &voxel_map_instances);
	MergeSubMaps(voxel_map_instances, voxel_map_.get());

	return true;
}

template <typename T>
void VoxelMapAveraging<T>::RemoveSamples(
	const std::vector<geometry::Point<T>>& points,
	const std::vector<T>& weights
	) {
	std::vector<VoxelMapAbstract<T, VoxelAvg>> voxel_map_instances;
	GenerateSubMaps(points, weights, voxel_size_, hash_range_, &voxel_map_instances);
	VoxelMapAbstract<T, VoxelAvg> vox_map_with_points_to_remove(voxel_size_, hash_range_);
	MergeSubMaps(voxel_map_instances, &vox_map_with_points_to_remove);
	voxel_map_->SubtractMap(vox_map_with_points_to_remove);
}


template <typename T>
std::array<std::unique_ptr<std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>>, 2> VoxelMapAveraging<T>::ExtractAllPoints(
	const T min_weight
	) const {
	std::unique_ptr<std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>> all_points(
		new std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>);
	std::unique_ptr<std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>> all_colors(
		new std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>);

	const std::unordered_map<int64_t, VoxelAvg>& hashmap = voxel_map_->GetMap();
	for(const auto& a : hashmap) {
		if(a.second.n_ >= min_weight) {
			all_points->push_back(a.second.xyz_);
			all_colors->push_back(a.second.rgb_);
		}
	}

	return {std::move(all_points), std::move(all_colors)};
}

template <typename T>
T VoxelMapAveraging<T>::GetWeight(const Eigen::Matrix<T, 3, 1>& point) const {
	const int64_t voxid = voxel_map_->GetVoxelId(point);
	return voxel_map_->GetMap().at(voxid).n_;
}

template <typename T>
T VoxelMapAveraging<T>::GetWeight(const geometry::Point<T>& point) const {
	const Eigen::Matrix<T, 3, 1> xyz = point.xyz_.template block<3,1>(0,0);
	const int64_t voxid = voxel_map_->GetVoxelId(xyz);
	return voxel_map_->GetMap().at(voxid).n_;
}

template <typename T>
const VoxelMapAbstract<T, typename VoxelMapAveraging<T>::VoxelAvg>& VoxelMapAveraging<T>::VoxMap() const {
	return *voxel_map_;
}

template class VoxelMapAveraging<float>;
template class VoxelMapAveraging<double>;

} // namespace voxel_map
