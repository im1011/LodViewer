#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace octree_reader {

///
/// Class that provides an interface to retrieve the binary file offsets to read the .octree file.
///
class OctreeReader {
public:
	///
	/// Contructor.
	/// Computes the content of the member variables.
	///
	OctreeReader(const std::string& octree_file);

	///
	/// Returns the file name.
	///
	std::string GetBinFileName() const;

	///
	/// Returns a set of all available hashes
	///
	std::unordered_set<uint64_t> AllHashes() const;

	///
	/// Offset accesssor.
	///
	size_t GetOffset(
		const size_t level,
		const uint64_t hash
		) const;

	///
	/// Size accesssor.
	///
	size_t GetSize(
		const size_t level,
		const uint64_t hash
		) const;

private:
	const std::string octree_file_;
	std::vector<std::unordered_map<uint64_t, size_t>> octree_offsets_;
	std::vector<std::unordered_map<uint64_t, size_t>> octree_sizes_;
};

} // namespace octree_reader