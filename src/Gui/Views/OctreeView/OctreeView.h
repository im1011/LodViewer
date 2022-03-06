#pragma once

#include <string>
#include <memory>
#include <thread>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <Gui/Views/ViewBase.h>
#include <Gui/Views/PointCloudView/PointCloudView.h>
#include <FileIO/OctreeReader.h>

namespace gui {

class OctreeView final : public ViewBase {
public:
	///
	/// Virtual destructor.
	///
	virtual ~OctreeView();

	///
	/// Constructor requires folder on disk.
	/// Voxel size referes to level 0 voxel size.
	/// rendered within a lod-voxel to make smoother lod transitions. 
	/// Allows the definition of screen resolution adjustment of the LOD.
	///
	OctreeView(
		const octree_reader::OctreeReader& octree_reader,
		const size_t num_pixels = 1920 * 1080,
		const float voxel_size = 10.0f
		) : octree_reader_(octree_reader),
			voxel_size_(voxel_size),
			resolution_adjustment_(ComputeResolutionAdjustment(num_pixels)) {
			};

	///
	/// Implements drawing. 
	///
	virtual void Draw(
		const Eigen::Matrix<float, 4, 4>& projection,
		const Eigen::Matrix<float, 4, 4>& w2v_tf
		) final override;

	///
	/// Initializes the view. 
	/// Intended to initialize buffers, the shader, etc.
	/// Also starts the thread that checks and handles new poses
	///
	virtual void Init() final override;

	///
	/// Sets point size for this view.
	///
	void SetPointSize(const float point_size);

	///
	/// Returns the lowest level of the octree currently active
	///
	size_t GetLowestLevel() const;

private:
	///
	/// Function designed to run in its own thread.
	///
	void LoadOctree();

	///
	/// Compues the resolution adjustment for the level switching
	///
	static float ComputeResolutionAdjustment(const size_t num_pixels); 

private:
    const octree_reader::OctreeReader& octree_reader_;
	const float voxel_size_;
	const float resolution_adjustment_;
	std::vector<std::unique_ptr<PointCloudView>> pc_views_;
	std::vector<size_t> pc_views_active_level_;
	std::vector<int64_t> pc_views_block_id_;
	std::vector<Eigen::Matrix<float, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 1>>> pc_views_centers_;
	std::unique_ptr<PointCloudView> pc_level_0_;

	// variables handling the octree loading work
	std::unique_ptr<std::thread> octree_load_thread_;
	bool entered_class_destructor_ = false;
	bool new_view_position_available_ = false;
	Eigen::Matrix<float, 4, 1> view_position_;
};

} // namespace gui
