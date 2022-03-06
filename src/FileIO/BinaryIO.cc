#include "BinaryIO.h"

namespace binary_io {

BinaryReader::BinaryReader(const std::string& file) {
	ifs_.open(file, std::ios::in | std::ios::binary);
}

BinaryReader::~BinaryReader() {
	ifs_.close();
}

template <typename T> 
bool BinaryReader::Read(T* const out) {
	ifs_.read(reinterpret_cast<char*>(out), static_cast<int64_t>(sizeof(T)));
	return ifs_.good();
}

template <typename T> 
T BinaryReader::Read() {
	T out;
	ifs_.read(reinterpret_cast<char*>(&out), static_cast<int64_t>(sizeof(T)));
	return out;
}

template <typename T> 
void BinaryReader::ReadN(const size_t n, T* const buf) {
	ifs_.read(reinterpret_cast<char*>(buf), static_cast<int64_t>(sizeof(T) * n));
}

std::string BinaryReader::ReadString(const size_t n) {
	std::string out(n, ' ');
	ReadN<char>(n, &(out[0]));
	return out;
}

void BinaryReader::Seek(const size_t pos) {
	ifs_.seekg(static_cast<std::streamoff>(pos));
}

BinaryWriter::BinaryWriter(const std::string& file, const bool append) {
	if(append)
		ofs_.open(file, std::ios::out | std::ios::binary | std::ios_base::app);
	else 
		ofs_.open(file, std::ios::out | std::ios::binary);
}

BinaryWriter::~BinaryWriter() {
	ofs_.close();
}

template <typename T> 
void BinaryWriter::Write(const T val) {
	ofs_.write(reinterpret_cast<const char*>(&val), static_cast<int64_t>(sizeof(T)));
}

template bool BinaryReader::Read<float>(float* const);
template bool BinaryReader::Read<double>(double* const);
template bool BinaryReader::Read<char>(char* const);
template bool BinaryReader::Read<uint8_t>(uint8_t* const);
template bool BinaryReader::Read<uint32_t>(uint32_t* const);
template bool BinaryReader::Read<uint16_t>(uint16_t* const);
template bool BinaryReader::Read<uint64_t>(uint64_t* const);
template bool BinaryReader::Read<int8_t>(int8_t* const);
template bool BinaryReader::Read<int32_t>(int32_t* const);
template bool BinaryReader::Read<int16_t>(int16_t* const);
template bool BinaryReader::Read<int64_t>(int64_t* const);

template float BinaryReader::Read<float>();
template double BinaryReader::Read<double>();
template char BinaryReader::Read<char>();
template uint8_t BinaryReader::Read<uint8_t>();
template uint32_t BinaryReader::Read<uint32_t>();
template uint16_t BinaryReader::Read<uint16_t>();
template uint64_t BinaryReader::Read<uint64_t>();
template int8_t BinaryReader::Read<int8_t>();
template int32_t BinaryReader::Read<int32_t>();
template int16_t BinaryReader::Read<int16_t>();
template int64_t BinaryReader::Read<int64_t>();

template void BinaryReader::ReadN<float>(const size_t, float* const);
template void BinaryReader::ReadN<double>(const size_t, double* const);
template void BinaryReader::ReadN<char>(const size_t, char* const);
template void BinaryReader::ReadN<uint8_t>(const size_t, uint8_t* const);
template void BinaryReader::ReadN<uint32_t>(const size_t, uint32_t* const);
template void BinaryReader::ReadN<uint16_t>(const size_t, uint16_t* const);
template void BinaryReader::ReadN<uint64_t>(const size_t, uint64_t* const);
template void BinaryReader::ReadN<int8_t>(const size_t, int8_t* const);
template void BinaryReader::ReadN<int32_t>(const size_t, int32_t* const);
template void BinaryReader::ReadN<int16_t>(const size_t, int16_t* const);
template void BinaryReader::ReadN<int64_t>(const size_t, int64_t* const);

template void BinaryWriter::Write<float>(const float);
template void BinaryWriter::Write<double>(const double);
template void BinaryWriter::Write<char>(const char);
template void BinaryWriter::Write<uint8_t>(const uint8_t);
template void BinaryWriter::Write<uint32_t>(const uint32_t);
template void BinaryWriter::Write<uint16_t>(const uint16_t);
template void BinaryWriter::Write<uint64_t>(const uint64_t);
template void BinaryWriter::Write<int8_t>(const int8_t);
template void BinaryWriter::Write<int32_t>(const int32_t);
template void BinaryWriter::Write<int16_t>(const int16_t);
template void BinaryWriter::Write<int64_t>(const int64_t);

} // namespace binary_io


