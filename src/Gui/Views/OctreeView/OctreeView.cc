#include "OctreeView.h"

#include <array>

#include <FileIO/BinaryIO.h>

namespace {

std::array<int64_t, 3> GetVoxelPosition(const int64_t id) {
	return {
		(id % (2ll * 100000ll)) - 100000ll,
		(id % (4ll * 100000ll * 100000ll)) / (2ll * 100000ll) - 100000ll,
		(id / (4ll * 100000ll * 100000ll)) - 100000ll
	};
}

Eigen::Matrix<float, 4, 1> GetVoxelCenter(
	const std::array<int64_t, 3>& position,
	const float voxel_size
	) {
	return Eigen::Matrix<float, 4, 1>(
		static_cast<float>(position[0]) * voxel_size + 0.5f * voxel_size, 
		static_cast<float>(position[1]) * voxel_size + 0.5f * voxel_size, 
		static_cast<float>(position[2]) * voxel_size + 0.5f * voxel_size,
		1.0f
	);
}

Eigen::Matrix<float, 4, 1> GetVoxelCenter(
	const int64_t id,
	const float voxel_size
	) {
	const std::array<int64_t, 3> position = GetVoxelPosition(id);
	return GetVoxelCenter(position, voxel_size);
}

void ReadPoints(
	const std::string& bin_file,
	const size_t offset,
	const size_t num_bytes,
	std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>* const points,
	std::vector<std::array<uint8_t, 4>>* const colors 
	) {
	binary_io::BinaryReader bin(bin_file);
	bin.Seek(offset);

	Eigen::Matrix<float, 4, 1> xyz;
	xyz(3) = 1.0f;
	std::array<uint8_t, 4> rgb;
	rgb[3] = 255;

	size_t num_bytes_read = 0;

	while(
		bin.Read<float>(&(xyz(0))) 
		&& bin.Read<float>(&(xyz(1))) 
		&& bin.Read<float>(&(xyz(2))) 
		&& bin.Read<uint8_t>(&(rgb[0])) 
		&& bin.Read<uint8_t>(&(rgb[1])) 
		&& bin.Read<uint8_t>(&(rgb[2]))
		) {
		points->push_back(xyz);
		colors->push_back(rgb);

		num_bytes_read += 3 * (sizeof(float) + sizeof(uint8_t));
		if(num_bytes_read == num_bytes)
			break;
	}
}

template <typename T>
Eigen::Matrix<T, 4, 4> InverseTransform(const Eigen::Matrix<T, 4, 4>& T4x4) {
	Eigen::Matrix<T, 3, 3> R = T4x4.template block<3,3>(0,0);
	Eigen::Matrix<T, 3, 1> t = T4x4.template block<3,1>(0,3);
	R.transposeInPlace();
	t = - R * t;

	Eigen::Matrix<T, 4, 4> T4x4_inv;
	T4x4_inv.template block<3,3>(0,0) = R;
	T4x4_inv.template block<3,1>(0,3) = t;
	T4x4_inv.template block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;
	return T4x4_inv;
}

} // namespace

namespace gui {

void OctreeView::Init() {
	for(const uint64_t hash : octree_reader_.AllHashes())
		pc_views_block_id_.push_back(static_cast<int64_t>(hash));
	const size_t num_blocks = pc_views_block_id_.size();

	pc_views_.resize(num_blocks);
	pc_views_active_level_.resize(num_blocks, 0);

	for(size_t i=0; i < num_blocks; ++i) {
		pc_views_[i].reset(new PointCloudView);
		pc_views_[i]->Initialize();
	}

	std::unique_ptr<std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>> all_l0_points(
		new std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>());
	std::unique_ptr<std::vector<std::array<uint8_t, 4>>> all_l0_colors(
		new std::vector<std::array<uint8_t, 4>>);

	for(size_t i=0; i < num_blocks; ++i) {
		std::unique_ptr<std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>> points(
			new std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>());
		std::unique_ptr<std::vector<std::array<uint8_t, 4>>> colors(
			new std::vector<std::array<uint8_t, 4>>);
		
		ReadPoints(
			octree_reader_.GetBinFileName(),
			octree_reader_.GetOffset(0, static_cast<uint64_t>(pc_views_block_id_[i])),
			octree_reader_.GetSize(0, static_cast<uint64_t>(pc_views_block_id_[i])),
			points.get(),
			colors.get()
			);

		all_l0_points->insert(all_l0_points->end(), points->begin(), points->end());
		all_l0_colors->insert(all_l0_colors->end(), colors->begin(), colors->end());

		pc_views_[i]->SetPoints(
			std::move(points),
			std::move(colors)
			);
		pc_views_[i]->SetHidden(true);
	}

	pc_views_centers_.resize(num_blocks);
	for(size_t i=0; i < num_blocks; ++i) 
		pc_views_centers_[i] = GetVoxelCenter(pc_views_block_id_[i], voxel_size_);

	pc_level_0_.reset(new PointCloudView);
	pc_level_0_->Initialize();
	pc_level_0_->SetPoints(
		std::move(all_l0_points),
		std::move(all_l0_colors)
		);

    octree_load_thread_.reset(new std::thread([&]() {
    	while(!entered_class_destructor_) {
    		while(this->IsHidden() && !entered_class_destructor_) 
    			std::this_thread::sleep_for(std::chrono::milliseconds(50));
	        this->LoadOctree();
    	}
    }));
}

void OctreeView::Draw(
	const Eigen::Matrix<float, 4, 4>& projection,
	const Eigen::Matrix<float, 4, 4>& w2v_tf
	) {
	if(this->IsHidden())
    	return;

	// check if level changes for any of the views
	const Eigen::Matrix<float, 4, 4> v2w = InverseTransform<float>(w2v_tf);
	view_position_ = v2w.block<4,1>(0,3);
	new_view_position_available_ = true;

	for(size_t i=0; i < pc_views_.size(); ++i)
		pc_views_[i]->Draw(projection, w2v_tf);
	pc_level_0_->Draw(projection, w2v_tf);
}

void OctreeView::SetPointSize(const float point_size) {
	for(size_t i=0; i < pc_views_.size(); ++i)
		pc_views_[i]->SetPointSize(point_size);
	pc_level_0_->SetPointSize(point_size);
}

void OctreeView::LoadOctree() {
	if(!new_view_position_available_)
		return;
	new_view_position_available_ = false;

	/*
	note:
	const double constant = 1638570.0; // solve for 7
	const double constant = 409600.0; // solve for 6
	const double constant = 819200.0; // solve for 6.5
	*/

	// set constant such that it fulfills a boundary condition for the level computation
	// here we say that anything closer than 10m is full level 6 (so lvl 7)
	const double constant = 1638570.0; 

	for(size_t i=0; i < pc_views_.size(); ++i) {
		const double dist_squared = static_cast<double>((pc_views_centers_[i] - view_position_).squaredNorm());
		const float level = static_cast<float>(0.7213475 * std::log(constant / dist_squared) + resolution_adjustment_);
		const size_t discrete_level = (level < 0.0f ? 0 : std::min(static_cast<size_t>(level), static_cast<size_t>(6)));

		if(discrete_level != pc_views_active_level_[i]) {
			pc_views_active_level_[i] = discrete_level;

			if(discrete_level == 0) {
				pc_views_[i]->SetHidden(true);
			} else {
				std::unique_ptr<std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>> points(
					new std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>>());
				std::unique_ptr<std::vector<std::array<uint8_t, 4>>> colors(
					new std::vector<std::array<uint8_t, 4>>);
				
				ReadPoints(
					octree_reader_.GetBinFileName(),
					octree_reader_.GetOffset(discrete_level, static_cast<uint64_t>(pc_views_block_id_[i])),
					octree_reader_.GetSize(discrete_level, static_cast<uint64_t>(pc_views_block_id_[i])),
					points.get(),
					colors.get()
					);

				pc_views_[i]->SetPoints(
					std::move(points),
					std::move(colors)
					);
				pc_views_[i]->SetHidden(false);
			}
		}
	}
}

OctreeView::~OctreeView() {
	entered_class_destructor_ = true;
	octree_load_thread_->join();
}

size_t OctreeView::GetLowestLevel() const {
	size_t max_level = 0;
	for(const size_t i : pc_views_active_level_)
		max_level = std::max(max_level, i);
	return max_level;
}

float OctreeView::ComputeResolutionAdjustment(const size_t num_pixels) {
    return static_cast<float>(0.7213475 * std::log(static_cast<double>(num_pixels) / 1920.0 / 1080.0));
}

} // namespace gui
