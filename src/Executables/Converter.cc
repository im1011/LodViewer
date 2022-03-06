#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <random>
#include <filesystem>

#include <gflags/gflags.h>

#include <FileIO/BinaryIO.h>
#include <FileIO/PlyIO.h>
#include <VoxelMap/VoxelMapAveraging.h>
#include <VoxelMap/KeyGenerate.h>

DEFINE_string(input_ply_file, "", "required");
DEFINE_string(output_octree_file, "", "required");
DEFINE_string(cache_folder, "", "required");

namespace {

///
/// Insert points and extract them in slightly structured order based on voxel sizes.
/// Sweeps the implied voxels sequencially one point at a time until all inserted points are returned.
///
class StructuredRandomOrder {
public:
	StructuredRandomOrder(
		const float voxel_size,
		const int64_t hash_range = 100000
		) : key_gen_(voxel_size, hash_range) {
	}

	void Insert(const std::pair<std::array<float, 3>, std::array<uint8_t,3>>& sample) {
		const int64_t key = key_gen_.GetVoxelId(Eigen::Matrix<float, 4, 1>(
			sample.first[0], sample.first[1], sample.first[2], 1.0));
		if(points_container_.find(key) == points_container_.end())
			points_container_.insert({key, std::vector<std::pair<std::array<float, 3>, std::array<uint8_t,3>>>()});
		points_container_.at(key).push_back(sample);
	}

	void Shuffle() {
		auto rng = std::default_random_engine {};
		for(auto& vec : points_container_)
			std::shuffle(vec.second.begin(), vec.second.end(), rng);
	}

	///
	/// Iterates through the points and calls the callback. 
	/// After this function, the internal container becomes empty.
	///
	void ExtractPoints(std::function<void(std::array<float, 3>, std::array<uint8_t,3>)> PointCallback) {
		std::unordered_set<int64_t> valid_keys;
		for(const auto& a : points_container_)
			valid_keys.insert(a.first);

		while(!valid_keys.empty()) {
			std::vector<int64_t> keys_that_became_empty;

			for(const int64_t key : valid_keys) {
				PointCallback(points_container_.at(key).back().first, points_container_.at(key).back().second);
				points_container_.at(key).pop_back();
				
				if(points_container_.at(key).empty())
					keys_that_became_empty.push_back(key);
			}

			for(const int64_t key : keys_that_became_empty)
				valid_keys.erase(key);
		}

		points_container_.clear();
	}

private:
	const voxel_map::KeyGenerate<float> key_gen_;
	std::unordered_map<int64_t, std::vector<std::pair<std::array<float, 3>, std::array<uint8_t,3>>>> points_container_;
};

///
/// Class that converts a ply file to a viewer-compatible octree file.
///
class Converter {
public:
	///
	/// Step that uses voxelmaps to create the various files for the various levels of the octree.
	///
	static void CreateHashedFiles(
			const std::string& ply_file,
			const std::string& cache_folder,
			const size_t level_to_become_level_zero,
			const size_t num_levels
		) {
		const std::vector<std::string> in_files = {
			ply_file
		};

		const std::vector<std::string> out_folders = {
			cache_folder + (cache_folder.back() != '/' ? "/" : "") + "octree_hash_files/"
		};

		for(const std::string& f : out_folders) {
			if(std::filesystem::exists(f))
				std::filesystem::remove_all(f);
			std::filesystem::create_directories(f);
		}


		const size_t chunk_size = 10000;
		const float level_0_voxel_size = 10.0f;
		const float structured_random_order_voxel_size = 2.5f;
		voxel_map::KeyGenerate<float> key_gen(level_0_voxel_size);

		std::vector<float> voxel_sizes(num_levels);
		voxel_sizes[0] = level_0_voxel_size;
		for(size_t i=1; i < voxel_sizes.size(); ++i)
			voxel_sizes[i] = 0.5f * voxel_sizes[i-1];

		for(size_t f = 0; f < in_files.size(); ++f) {
			// first step is to split the big bin file into smaller bin files, each for its own L0
			const std::string bin_file_chunk_folder = cache_folder + "points_splitting_" + std::to_string(f) + "/";
			std::unordered_set<int64_t> bin_file_chunk_keys;
			{
				if(std::filesystem::exists(bin_file_chunk_folder))
					std::filesystem::remove_all(bin_file_chunk_folder);
				std::filesystem::create_directory(bin_file_chunk_folder);
				
				std::vector<geometry::Point<float>> points;
				ply_io::PlyIO<float>::ReadPly(in_files[f], &points);

				Eigen::Matrix<double, 3, 1> average_xyz_double = Eigen::Matrix<double, 3, 1>::Zero();
				double num_samples = 0.0;
				for(const geometry::Point<float>& point : points) {
					++num_samples;
					average_xyz_double += (point.Cast<double>().xyz_.block<3,1>(0,0) - average_xyz_double) / num_samples;
				}
				const Eigen::Matrix<float, 3, 1> average_xyz_float = average_xyz_double.cast<float>();

				for(geometry::Point<float>& point : points)
					point.xyz_.block<3,1>(0,0) -= average_xyz_float;

				for(const geometry::Point<float>& point : points) {
					const int64_t key = key_gen.GetVoxelId(point.xyz_);
					bin_file_chunk_keys.insert(key);
					binary_io::BinaryWriter writer_append(bin_file_chunk_folder + std::to_string(key) + ".bin", true);
					writer_append.Write<float>(point.xyz_(0));
					writer_append.Write<float>(point.xyz_(1));
					writer_append.Write<float>(point.xyz_(2));
					writer_append.Write<uint8_t>(point.c_[0]);
					writer_append.Write<uint8_t>(point.c_[1]);
					writer_append.Write<uint8_t>(point.c_[2]);
				}
			}

			// second step is to apply voxmaps on the chunks
			std::vector<int64_t> bin_file_chunk_keys_vector;
			for(const int64_t key : bin_file_chunk_keys)
				bin_file_chunk_keys_vector.push_back(key);

			#pragma omp parallel for
			for(size_t i = 0; i < bin_file_chunk_keys_vector.size(); ++i) {
				const int64_t key = bin_file_chunk_keys_vector[i];
				binary_io::BinaryReader bin(bin_file_chunk_folder + std::to_string(key) + ".bin");

				std::vector<std::unique_ptr<voxel_map::VoxelMapAveraging<float>>> voxmaps(num_levels);
				for(size_t i=0; i < num_levels; ++i)
					voxmaps[i].reset(new voxel_map::VoxelMapAveraging<float>(voxel_sizes[i]));

				std::vector<geometry::Point<float>> insertion_chunk;
				geometry::Point<float> point;
				while(
					bin.Read<float>(&(point.xyz_(0))) 
					&& bin.Read<float>(&(point.xyz_(1))) 
					&& bin.Read<float>(&(point.xyz_(2))) 
					&& bin.Read<uint8_t>(&(point.c_[0])) 
					&& bin.Read<uint8_t>(&(point.c_[1])) 
					&& bin.Read<uint8_t>(&(point.c_[2]))
					) {
					insertion_chunk.push_back(point);
					if(insertion_chunk.size() == chunk_size) {
						for(size_t i=0; i < num_levels; ++i)
							voxmaps[i]->AddSamples(insertion_chunk);
						insertion_chunk.clear();
					}
				}
				for(size_t i=0; i < num_levels; ++i)
					voxmaps[i]->AddSamples(insertion_chunk);
				insertion_chunk.clear();

				// part that writes the bin files
				for(size_t i = level_to_become_level_zero; i < num_levels; ++i) {
					std::array<std::unique_ptr<std::vector<Eigen::Matrix<float, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 1>>>>, 2> xyz_rgb 
						= voxmaps[i]->ExtractAllPoints();

					StructuredRandomOrder structured_random_ordering(structured_random_order_voxel_size);
					for(size_t j=0; j < xyz_rgb[0]->size(); ++j) {
						structured_random_ordering.Insert({{
							xyz_rgb[0]->at(j)(0),
							xyz_rgb[0]->at(j)(1),
							xyz_rgb[0]->at(j)(2)
						}, {
							static_cast<uint8_t>(xyz_rgb[1]->at(j)(0)),
							static_cast<uint8_t>(xyz_rgb[1]->at(j)(1)),
							static_cast<uint8_t>(xyz_rgb[1]->at(j)(2))
						}});
					}
					structured_random_ordering.Shuffle();

					binary_io::BinaryWriter writer(out_folders[f] + std::to_string(i - level_to_become_level_zero) + std::to_string(key) + ".bin");
					structured_random_ordering.ExtractPoints([&](std::array<float, 3> xyz, std::array<uint8_t, 3> rgb){
						writer.Write<float>(xyz[0]);
						writer.Write<float>(xyz[1]);
						writer.Write<float>(xyz[2]);
						writer.Write<uint8_t>(rgb[0]);
						writer.Write<uint8_t>(rgb[1]);
						writer.Write<uint8_t>(rgb[2]);
					});
				}
			}
		}
	}

	///
	/// Generates single file from the individual octree files in the cache.
	///
	static void FileBundling(
		const std::string& cache_folder,
		const std::string& output_file,
		const size_t num_levels
		) {
		const std::string octree_dir = cache_folder + (cache_folder.back() != '/' ? "/" : "") + "octree_hash_files/";
		binary_io::BinaryWriter bin_writer(output_file);

		std::vector<std::string> octree_bin_files;
		GetDirFilesWithExtention(octree_dir, ".bin", &octree_bin_files);

		std::vector<std::unordered_map<uint64_t, size_t>> relative_file_offsets(num_levels);
		std::vector<std::unordered_map<uint64_t, size_t>> file_sizes(num_levels);
		size_t relative_total_size = 0;

		for(const std::string& bin : octree_bin_files) {
			const std::string bin_path = octree_dir + "/" + bin;
			const size_t level = GetIntsFromString(bin.substr(0,1))[0];
			const uint64_t hash = GetIntsFromString(bin.substr(1))[0];

			relative_file_offsets[level].insert({hash, relative_total_size});
			const size_t this_file_size = std::filesystem::file_size(bin_path);
			relative_total_size += this_file_size;
			file_sizes[level].insert({hash, this_file_size});
		}

		size_t octree_header_size = 0;
		for(size_t j=0; j < num_levels; ++j) {
			octree_header_size += sizeof(size_t);
			octree_header_size += relative_file_offsets[j].size() * sizeof(uint64_t);
			octree_header_size += relative_file_offsets[j].size() * sizeof(size_t);
			octree_header_size += relative_file_offsets[j].size() * sizeof(size_t);
		}

		size_t total_size = 0;
		for(size_t j=0; j < num_levels; ++j) {
			bin_writer.Write<size_t>(relative_file_offsets[j].size());
			for(const auto& a : relative_file_offsets[j]) {
				bin_writer.Write<uint64_t>(a.first);
				bin_writer.Write<size_t>(a.second + total_size + octree_header_size);
				bin_writer.Write<size_t>(file_sizes[j].at(a.first));
			}
		}
		total_size += octree_header_size;

		for(const std::string& bin : octree_bin_files) {
			const std::string bin_path = octree_dir + "/" + bin;
			const size_t bin_size = std::filesystem::file_size(bin_path);
			
			uint8_t single_byte;
			binary_io::BinaryReader bin_reader(bin_path);
			while(bin_reader.Read<uint8_t>(&single_byte))
				bin_writer.Write<uint8_t>(single_byte);
			total_size += bin_size;
		}
	}

private:
	///
	/// Wrapper around std filesystem function.
	///
	static bool GetDirFiles(
	  	const std::string& dir, 
	  	std::vector<std::string>* const files
	  	) {
	  	files->clear();

	  	for(const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(dir)) {
	    	if (entry.is_regular_file())
	        	files->push_back(entry.path().filename().string());
	  	}
	  	return true;
	}

	///
	/// only works if the dir names dont have a dot
	///
	static std::vector<std::string> GetDirFilesRecursively(const std::string& dir) {
	    std::vector<std::string> files;
	    GetDirFiles(dir, &files);

	    std::vector<std::string> out_files;

	    for(const std::string f : files) {
			std::vector<std::string> splitted = Split(f, '.');
			if(splitted.size() == 1) {
				std::vector<std::string> sub_files = GetDirFilesRecursively(dir + f + "/");
				for(std::string sf : sub_files)
					out_files.push_back(sf);
			} else {
				out_files.push_back(dir + f);
			}
	    }
	    return out_files;
	}

	///
	/// Recursively searches the directory.
	/// 
	static bool GetDirFilesWithExtention(
		const std::string& dir, 
		const std::string& extention, 
		std::vector<std::string>* const files
		) {
		files->clear();
		std::vector<std::string> all_files_in_dir;

		const bool ret = GetDirFiles(dir, &all_files_in_dir);
		if(!ret)
			return false;

		for(const std::string & f : all_files_in_dir) {
			if(f.size() < extention.size())
				continue;

			const std::string last_chars_f = f.substr(f.size() - extention.size());
			if(last_chars_f.compare(extention) == 0)
				files->push_back(f);
		}

		std::sort(files->begin(), files->end());
			return true;
	}

	///
	/// Places integers within the string into the vector. Function is unsigned.
	///
	static std::vector<size_t> GetIntsFromString(const std::string& str) {
		std::vector<size_t> out;

		size_t curr = 0;
		bool valid = false;

		for(const char c : str) {
			if(c >= '0' && c <= '9') {
				curr *= 10;
				curr += static_cast<size_t>(c - '0');
				valid = true;
			} else if(valid) {
				out.push_back(curr);
				curr = 0;
				valid = false;
			}
		}

		if(valid)
		out.push_back(curr);

		return out;
	}

	///
	/// Splits a string based on the delimiter into a vector of strings.
	///
	static std::vector<std::string> Split(
		const std::string & string_to_split,
		const char delimiter
	) {
		std::istringstream ss(string_to_split);
		std::string token;

		std::vector<std::string> output;

		while(std::getline(ss, token, delimiter)) 
			output.push_back(token);

		// get rid of trailing null
		if(output.back().back() == '\n')
			output.back().pop_back();

		return output;
	}
};


} // namespace

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
	
	const size_t level_to_become_level_zero = 3;
	const size_t highest_level = 9;

	Converter::CreateHashedFiles(FLAGS_input_ply_file, FLAGS_cache_folder, 
		level_to_become_level_zero, highest_level + 1);
	Converter::FileBundling(FLAGS_cache_folder, FLAGS_output_octree_file, 
		highest_level - level_to_become_level_zero + 1);

    return 0;
}
