#pragma once

#include <fstream>
#include <cstring>

namespace binary_io {

///
/// Helper class to read from general binary files.
///
class BinaryReader {
public:
	BinaryReader(const std::string& file);
	~BinaryReader();

	///
	/// This returns the success of the read operation. If false is returned then the file is completely read.
	///
	template <typename T> 
	bool Read(T* const out);

	///
	/// This assumes the caller knows that a value can be read.
	///
	template <typename T> 
	T Read();

	///
	/// Reads n elements of the same type.
	/// This assumes the caller knows that a value can be read.
	///
	template <typename T> 
	void ReadN(const size_t n, T* const buf);

	///
	/// Reads an n-character string.
	/// This assumes the caller knows that a value can be read.
	///
	std::string ReadString(const size_t n);

	///
	/// Calls the std::istream::seekg function to implement reading with offsets.
	///
	void Seek(const size_t pos);

private:
	std::ifstream ifs_;
};

///
/// Helper class to write general binary files.
///
class BinaryWriter {
public:
	BinaryWriter(const std::string& file, const bool append = false);
	~BinaryWriter();

	///
	/// Writes the value with the specified template type.
	///
	template <typename T> 
	void Write(const T val);
private:
	std::ofstream ofs_;
};

} // namespace binary_io
