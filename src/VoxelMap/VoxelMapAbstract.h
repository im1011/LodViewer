#pragma once

#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <array>
#include <list>

namespace voxel_map { 

///
/// Hashtable based implementation of abstact voxelgrid
/// 2nd template parameter specifies the voxel class with following members:
/// + template<typename R> bool Insert(R, const Eigen::Matrix<S, 3, 1>&) for types R where VoxelMapAbstract is used
/// + (optional if mergemap is used) bool Insert(const TVoxel& vox_content)
/// + (optional if subtractmap is used) bool Subtract(const TVoxel& vox_content) -> returning if voxel has been zeroed after subtraction
/// + must have default constructor
///	
template<typename TFloat, class TVoxel>
class VoxelMapAbstract {
public:
	///
	/// Constructor. 
	///
	VoxelMapAbstract(
		const TFloat voxel_size,
		const int64_t hash_range = 100000, // how many map elements can fit into the hashtable in either direction +x/-x/+y/-y/+z/-z |---> [-R, R[
		const std::function<void()>& out_of_hash_range_callback = {}
		) : voxel_size_(voxel_size), 
			inverse_voxel_size_(static_cast<TFloat>(1.0) / voxel_size), 
			hash_range_(static_cast<int64_t>(hash_range)), 
			out_of_hash_range_callback_(out_of_hash_range_callback) {
	}

	///
	/// Returns indeces of the voxel containing the input point.
	///
	std::array<int64_t, 3> GetVoxelPosition(const Eigen::Matrix<TFloat, 3, 1>& xyz) const {
		return {
			static_cast<int64_t>(xyz(0) * inverse_voxel_size_) + (xyz(0) < 0.0 ? -1 : 0),
			static_cast<int64_t>(xyz(1) * inverse_voxel_size_) + (xyz(1) < 0.0 ? -1 : 0),
			static_cast<int64_t>(xyz(2) * inverse_voxel_size_) + (xyz(2) < 0.0 ? -1 : 0)
		};
	}

	///
	/// Convertes serial index to 3d voxel index.
	///
	std::array<int64_t, 3> GetVoxelPosition(const int64_t id) const {
		return {
			(id % (2 * hash_range_)) - hash_range_,
			(id % (4 * hash_range_ * hash_range_)) / (2 * hash_range_) - hash_range_,
			(id / (4 * hash_range_ * hash_range_)) - hash_range_
		};
	}

	///
	/// Returns serial id of the voxel containing the input point.
	///
	int64_t GetVoxelId(const Eigen::Matrix<TFloat, 3, 1>& xyz) const {
		const std::array<int64_t, 3> ijk = GetVoxelPosition(xyz);
		const int64_t i = ijk[0];
		const int64_t j = ijk[1];
		const int64_t k = ijk[2];

		if(out_of_hash_range_callback_) {
			if( i+hash_range_ < 0 || i+hash_range_ >= 2*hash_range_ ) out_of_hash_range_callback_();
			if( j+hash_range_ < 0 || j+hash_range_ >= 2*hash_range_ ) out_of_hash_range_callback_();
			if( k+hash_range_ < 0 || k+hash_range_ >= 2*hash_range_ ) out_of_hash_range_callback_();
		}

		return GetVoxelId(i, j, k);
	}

	///
	/// Overload. Returns serial id of the voxel containing the input point.
	///
	int64_t GetVoxelId(const Eigen::Matrix<TFloat, 4, 1>& xyz1) const {
		return GetVoxelId(xyz1.block<3,1>(0,0));
	}

	///
	/// Converts 3d voxel index to serial index.
	///
	int64_t GetVoxelId(const int64_t i, const int64_t j, const int64_t k) const {
		return (i+hash_range_) + 2*hash_range_*(j+hash_range_) + 4*hash_range_*hash_range_*(k+hash_range_);
	}

	///
	/// Overload. Converts 3d voxel index to serial index.
	///
	int64_t GetVoxelId(const std::array<int64_t, 3>& ijk) const {
		return GetVoxelId(ijk[0], ijk[1], ijk[2]);
	}

	///
	/// Returns center of voxel for the serial voxel index. 
	///
	Eigen::Matrix<TFloat, 3, 1> GetVoxelCenter(const int64_t id) const {
		const std::array<int64_t, 3> position = GetVoxelPosition(id);
		return GetVoxelCenter(position);
	}

	///
	/// Returns center of voxel for the 3d voxel position index. 
	///
	Eigen::Matrix<TFloat, 3, 1> GetVoxelCenter(const std::array<int64_t, 3>& position) const {
		return Eigen::Matrix<TFloat, 3, 1>(
			static_cast<TFloat>(position[0]) * voxel_size_ + static_cast<TFloat>(0.5) * voxel_size_, 
			static_cast<TFloat>(position[1]) * voxel_size_ + static_cast<TFloat>(0.5) * voxel_size_, 
			static_cast<TFloat>(position[2]) * voxel_size_ + static_cast<TFloat>(0.5) * voxel_size_
		);
	}

	///
	/// TDataSample must have member: "Eigen::Matrix<S, 3, 1> xyz_;"
	/// TVoxel must have member function bool Insert(TDataSample, const Eigen::Matrix<TFloat, 3, 1>&)
	///
	template<typename TDataSample>
	bool Insert(const TDataSample& data) {
		const std::array<int64_t, 3> vox_position = GetVoxelPosition(data.xyz_);
		const int64_t vox_id = GetVoxelId(vox_position);
		const Eigen::Matrix<TFloat, 3, 1> xyz_voxel_center = GetVoxelCenter(vox_position);

		if( map_.find(vox_id) == map_.end() )
			map_.insert(std::make_pair(vox_id, TVoxel()));

		const bool ret = map_.at(vox_id).Insert(data, xyz_voxel_center);
		return ret;
	}

	///
	/// TDataSample must have member: "Eigen::Matrix<S, 3, 1> xyz_;"
	///
	template<typename TDataSample>
	bool RegionalInsert(
		const TDataSample& data,
		const std::function<bool(const int64_t)> & insert_in_this_voxel
		) {
		std::list<std::array<int64_t, 3>> pending_voxel_positions;
		pending_voxel_positions.push_back(GetVoxelPosition(data.xyz_.template block<3,1>(0,0)));
		std::unordered_set<int64_t> visited_voxels;

		while(!pending_voxel_positions.empty())	{
			std::array<int64_t, 3> position_now = pending_voxel_positions.front();
			pending_voxel_positions.pop_front();

			if( out_of_hash_range_callback_ ) {
				if(position_now[0] + hash_range_ < 0 || position_now[0] + hash_range_ >= 2*hash_range_
				|| position_now[1] + hash_range_ < 0 || position_now[1] + hash_range_ >= 2*hash_range_
				|| position_now[2] + hash_range_ < 0 || position_now[2] + hash_range_ >= 2*hash_range_) {
					out_of_hash_range_callback_();
					return false;
				}
			}

			const int64_t vox_id_now = GetVoxelId(position_now);
			
			if( visited_voxels.find(vox_id_now) != visited_voxels.end() )
				continue; // this vox has already been visited
			visited_voxels.insert(vox_id_now);

			const Eigen::Matrix<TFloat, 3, 1> xyz_voxel_center = GetVoxelCenter(position_now);

			if(!insert_in_this_voxel(vox_id_now))
				continue;

			if(map_.find(vox_id_now) == map_.end())
				map_.insert(std::make_pair(vox_id_now, TVoxel()));
			
			const bool ret = map_.at(vox_id_now).Insert(data, xyz_voxel_center);
			if(!ret)
				return false;

			pending_voxel_positions.push_back({position_now[0] + 1, position_now[1], position_now[2]});
			pending_voxel_positions.push_back({position_now[0] - 1, position_now[1], position_now[2]});
			pending_voxel_positions.push_back({position_now[0], position_now[1] + 1, position_now[2]});
			pending_voxel_positions.push_back({position_now[0], position_now[1] - 1, position_now[2]});
			pending_voxel_positions.push_back({position_now[0], position_now[1], position_now[2] + 1});
			pending_voxel_positions.push_back({position_now[0], position_now[1], position_now[2] - 1});
		}
		return true;
	}

	///
	/// Merges another voxel-grid-map of the same signature into this one.
	///
	bool MergeMap(const VoxelMapAbstract<TFloat, TVoxel>& map) {
		const std::unordered_map<int64_t, TVoxel>& vox_map_hash_map = map.GetMap();
		for(const auto& id_vox_pair : vox_map_hash_map) {
			const int64_t vox_id = id_vox_pair.first;
			const TVoxel& vox_content = id_vox_pair.second;

			if( map_.find(vox_id) == map_.end() )
				map_.insert(std::make_pair(vox_id, TVoxel()));

			const bool ret = map_.at(vox_id).Insert(vox_content);
			
			if(!ret)
				return false;
		}
		return true;
	}

	///
	/// Subtracts another voxel-grid-map of the same signature from this one.
	/// If weigthts of voxels drop below close to zero, they are deallocated.
	///
	void SubtractMap(const VoxelMapAbstract<TFloat, TVoxel>& map) {
		const std::unordered_map<int64_t, TVoxel>& vox_map_hash_map = map.GetMap();
		for(const auto& id_vox_pair : vox_map_hash_map) {
			const int64_t vox_id = id_vox_pair.first;
			const TVoxel& vox_content = id_vox_pair.second;

			if( map_.find(vox_id) == map_.end() )
				continue;

			const bool ret_voxel_is_zeroed = map_.at(vox_id).Subtract(vox_content);
			
			if(ret_voxel_is_zeroed)
				map_.erase(vox_id);
		}
	}

	///
	/// Returns reference to the underlying hashtable data.
	///
	const std::unordered_map<int64_t, TVoxel>& GetMap() const {
		return map_;
	}

	///
	/// Returns mutable reference to the underlying hashtable data.
	///
	std::unordered_map<int64_t, TVoxel>* GetMapMutable() {
		return &map_;
	}

	///
	/// Checks if a voxel exists
	///
	bool VoxExists(const int64_t key) const {
		return (map_.find(key) != map_.end());
	}

	///
	/// Overload. Checks if a voxel exists
	///
	bool VoxExists(const std::array<int64_t, 3>& ijk) const {
		return VoxExists(GetVoxelId(ijk));
	}

	///
	/// Overload. Checks if a voxel exists
	///
	bool VoxExists(const int64_t i, const int64_t j, const int64_t k) const {
		return VoxExists(GetVoxelId(i, j, k));
	}

	///
	/// Accessor to single voxels. Assumes i,j,k are valid.
	///
	const TVoxel& Vox(const int64_t i, const int64_t j, const int64_t k) const {
		return map_.at(GetVoxelId(i, j, k));
	} 

private:
	const TFloat voxel_size_;
	const TFloat inverse_voxel_size_;
	const int64_t hash_range_;
	const std::function<void()>& out_of_hash_range_callback_;

	std::unordered_map<int64_t, TVoxel> map_;
};

} // namespace voxel_map
