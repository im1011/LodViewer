#include "OctreeReader.h"

#include <FileIO/BinaryIO.h>

namespace octree_reader {

OctreeReader::OctreeReader(const std::string& octree_file) : octree_file_(octree_file) {
	binary_io::BinaryReader bin_reader(octree_file);

	octree_offsets_.resize(7);
	octree_sizes_.resize(7);

	for(size_t j = 0; j < 7; ++j) {
		const size_t num_map_elements = bin_reader.Read<size_t>();
		for(size_t k = 0; k < num_map_elements; ++k) {
			const uint64_t hash = bin_reader.Read<uint64_t>();
			const size_t offset = bin_reader.Read<size_t>();
			const size_t size = bin_reader.Read<size_t>();
			octree_offsets_[j].insert({hash, offset});
			octree_sizes_[j].insert({hash, size});
		}
	}	
}

std::string OctreeReader::GetBinFileName() const {
	return octree_file_;
}

std::unordered_set<uint64_t> OctreeReader::AllHashes() const {
	std::unordered_set<uint64_t> all_hashes;
	for(const auto& a : octree_offsets_.at(0))
		all_hashes.insert(a.first);
	return all_hashes;
}

size_t OctreeReader::GetOffset(
		const size_t level,
		const uint64_t hash
		) const {
	return octree_offsets_.at(level).at(hash);
}

size_t OctreeReader::GetSize(
		const size_t level,
		const uint64_t hash
		) const {
	return octree_sizes_.at(level).at(hash);
}

} // namespace octree_reader