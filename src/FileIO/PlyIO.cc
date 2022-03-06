#include "PlyIO.h"

namespace {
	// TODO the standalone function shall become deprecated and replaced with the Binary IO class
	template <typename T>
	void AddDataToBinaryBlob(const T data, uint8_t * const binary_data_blob, size_t * const idx)
	{
		std::memcpy(&(binary_data_blob[*idx]), &data, sizeof(T));
		*idx += sizeof(T);
	}

	template <typename T>
	T GetDataFromBinaryBlob(const uint8_t * const binary_data_blob, size_t * const idx)
	{
		T data;
		std::memcpy(&data, &(binary_data_blob[*idx]), sizeof(T));
		*idx += sizeof(T);
		return data;
	}
} // namespace

namespace ply_io {

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const normals,
		const std::vector<std::array<uint8_t, 3> >* const colors,
		const std::vector<std::array<size_t, 3> >* const triangles,
		const std::vector<T>* const intensities
		) {
	// first, check that the sizes match
	if(normals != nullptr && points.size() != normals->size())
		return false;
	if(colors != nullptr && points.size() != colors->size())
		return false;
	if(intensities != nullptr && points.size() != intensities->size())
		return false;

	std::ofstream ofs(filename.c_str());

	ofs << "ply" << std::endl;
	ofs << "format binary_little_endian 1.0" << std::endl;
	ofs << "element vertex " << points.size() << std::endl;
	std::string precision_type;
	if(sizeof(T) == sizeof(float)) precision_type = "float";
	else if(sizeof(T) == sizeof(double)) precision_type = "double";
	else return false;
	ofs << "property " + precision_type + " x" << std::endl;
	ofs << "property " + precision_type + " y" << std::endl;
	ofs << "property " + precision_type + " z" << std::endl;
	if(normals != nullptr) {
		ofs << "property " + precision_type + " nx" << std::endl;
		ofs << "property " + precision_type + " ny" << std::endl;
		ofs << "property " + precision_type + " nz" << std::endl;
	}
	if(colors != nullptr) {
		ofs << "property uchar red" << std::endl;
		ofs << "property uchar green" << std::endl;
		ofs << "property uchar blue" << std::endl;
		ofs << "property uchar alpha" << std::endl;
	}
	if(intensities != nullptr) {
		ofs << "property " + precision_type + " intensity_value" << std::endl;
	}
	if(triangles != nullptr) {
		ofs << "element face " << triangles->size() << std::endl;
		ofs << "property list uchar int vertex_indices" << std::endl;
	}
	ofs << "end_header" << std::endl;


	size_t block_size_points = 3 * sizeof(T);
	if(normals != nullptr) block_size_points += 3 * sizeof(T);
	if(colors != nullptr) block_size_points += 4 * sizeof(uint8_t);
	if(intensities != nullptr) block_size_points += sizeof(T);

	size_t block_size_triangles = sizeof(uint8_t) + 3 * sizeof(int);

    std::string data;
    data.resize(points.size() * block_size_points + (triangles != nullptr ? triangles->size() : 0) * block_size_triangles);
    uint8_t * const data_ptr = reinterpret_cast<uint8_t * const>( &(data[0]) );
    size_t data_ptr_id = 0;

	for(size_t i=0; i < points.size(); ++i) {
		for(Eigen::Index j=0; j < 3; ++j)
			AddDataToBinaryBlob<T>(points[i](j), data_ptr, &data_ptr_id);
		
		if(normals != nullptr)
			for(Eigen::Index j=0; j < 3; ++j)
				AddDataToBinaryBlob<T>((*normals)[i](j), data_ptr, &data_ptr_id);
		
		if(colors != nullptr) {
			for(size_t j=0; j < 3; ++j)
				AddDataToBinaryBlob<uint8_t>((*colors)[i][j], data_ptr, &data_ptr_id);
			AddDataToBinaryBlob<uint8_t>(static_cast<uint8_t>(255), data_ptr, &data_ptr_id);
		}

		if(intensities != nullptr)
			AddDataToBinaryBlob<T>((*intensities)[i], data_ptr, &data_ptr_id);
	}

	if(triangles != nullptr) {
		for(size_t i=0; i < triangles->size(); ++i) {
			AddDataToBinaryBlob<uint8_t>(static_cast<uint8_t>(3), data_ptr, &data_ptr_id);
			for(size_t j=0; j < 3; ++j)
				AddDataToBinaryBlob<int>(static_cast<int>((*triangles)[i][j]), data_ptr, &data_ptr_id);
		}
	}

	ofs << data;

	ofs.close();
	return true;
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 4, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const normals,
		const std::vector<std::array<uint8_t, 3> >* const colors,
		const std::vector<std::array<size_t, 3> >* const triangles,
		const std::vector<T>* const intensities
		) {
	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> points_vec3(points.size());
	#pragma omp parallel for
	for(size_t i=0; i < points.size(); ++i) 
		points_vec3[i] = points[i].template block<3, 1>(0,0);
	return WritePly(filename, points_vec3, normals, colors, triangles, intensities);
}


template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals
		) {
	return WritePly(filename, points, &normals);
}


template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<std::array<uint8_t, 3> >& colors
		) {
	return WritePly(filename, points, nullptr, &colors);
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals,
		const std::vector<std::array<uint8_t, 3> >& colors
		) {
	return WritePly(filename, points, &normals, &colors);
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<std::array<size_t, 3> >& triangles
		) {
	return WritePly(filename, points, nullptr, nullptr, &triangles);
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<std::array<uint8_t, 3> >& colors,
		const std::vector<std::array<size_t, 3> >& triangles
		) {
	return WritePly(filename, points, nullptr, &colors, &triangles);
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals,
		const std::vector<std::array<size_t, 3> >& triangles
		) {
	return WritePly(filename, points, &normals, nullptr, &triangles);
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& points,
		const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>& normals,
		const std::vector<std::array<uint8_t, 3> >& colors,
		const std::vector<std::array<size_t, 3> >& triangles
		) {
	return WritePly(filename, points, &normals, &colors, &triangles);
}

template <typename T>
bool PlyIO<T>::WritePly(
		const std::string& filename,
		const std::vector<geometry::Point<T>>& point_structs,
		const std::vector<std::array<size_t, 3> >* const triangles
		) {
	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> points;
	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> normals;
	std::vector<std::array<uint8_t, 3> > colors;
	std::vector<T> intensities;

	for(size_t i=0; i < point_structs.size(); ++i) {
		points.push_back(Eigen::Matrix<T, 3, 1>(point_structs[i].xyz_(0), point_structs[i].xyz_(1), point_structs[i].xyz_(2)));
		intensities.push_back(point_structs[i].i_);
		if(point_structs[i].n_.squaredNorm() > 0.1)
			normals.push_back(point_structs[i].n_);
		if(point_structs[i].c_[3] != 0)
			colors.push_back({point_structs[i].c_[0], point_structs[i].c_[1], point_structs[i].c_[2]});
	}

	const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> * const normals_ptr = (normals.size() > 0 ? &normals : nullptr);
	const std::vector<std::array<uint8_t, 3>> * const colors_ptr = (colors.size() > 0 ? &colors : nullptr);
	const std::vector<T> * const intensities_ptr = (intensities.size() > 0 ? &intensities : nullptr);

	return WritePly(filename, points, normals_ptr, colors_ptr, triangles, intensities_ptr);
}

template <typename T>
bool PlyIO<T>::ReadPly(
		const std::string& filename,
		std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const output_points,
		std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>>* const output_normals,
		std::vector<std::array<uint8_t, 3> >* const output_colors,
		std::vector<std::array<size_t, 3> >* const output_triangles,
		std::vector<T> * const output_intensities
		) {
	std::ifstream ifs(filename.c_str(), std::ifstream::binary);

	bool file_contains_coordinates = false;
	bool file_contains_normals = false;
	bool file_contains_colors = false;
	bool file_contains_triangles = false;
	bool file_contains_intensities = false;

	size_t num_vertices = 0;
	size_t num_triangles = 0;


	if (ifs.is_open()) {
		while(ifs.good()) {
			std::string line;
			getline (ifs, line);
			if(line.empty())
				continue;

			std::stringstream ss(line);
			std::string first_word;
			ss >> first_word;

			if(first_word.compare("element") == 0) {
				std::string element_type;
				ss >> element_type;
				if(element_type.compare("vertex") == 0) {
					ss >> num_vertices;
				} else if(element_type.compare("face") == 0) {
					file_contains_triangles = true;
					ss >> num_triangles;
				} else {
					return false;
				}
			} else if(first_word.compare("property") == 0) {
				std::string second_word, third_word;
				ss >> second_word >> third_word;
				if(third_word.compare("x") == 0)
					file_contains_coordinates = true;
				else if(third_word.compare("nx") == 0)
					file_contains_normals = true;
				else if(third_word.compare("red") == 0)
					file_contains_colors = true;
				else if(third_word.compare("intensity_value") == 0)
					file_contains_intensities = true;
			} else if(first_word.compare("end_header") == 0) {
				break;
			}
		}
	} else {
		return false;
	}
	
	if(!file_contains_coordinates)
		return false;

	size_t block_size_points = 3 * sizeof(T);
	if(file_contains_normals)
		block_size_points += 3 * sizeof(T);
	if(file_contains_colors)
		block_size_points += 4 * sizeof(uint8_t);
	if(file_contains_intensities)
		block_size_points += sizeof(T);

	size_t block_size_triangles = sizeof(uint8_t) + 3 * sizeof(int);

	std::vector<char> raw_data(num_vertices * block_size_points + (file_contains_triangles ? num_triangles : 0) * block_size_triangles + 1);

	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> points;
	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> normals;
	std::vector<std::array<uint8_t, 3> > colors;
	std::vector<std::array<size_t, 3> > triangles;
	std::vector<T> intensities;

	points.resize(num_vertices);
	if(file_contains_normals)
		normals.resize(num_vertices);
	if(file_contains_colors)
		colors.resize(num_vertices);
	if(file_contains_triangles)
		triangles.resize(num_triangles);
	if(file_contains_intensities)
		intensities.resize(num_vertices);

	ifs.read(&(raw_data[0]), static_cast<int64_t>(raw_data.size()));
	ifs.close();

	const uint8_t * const data_ptr = reinterpret_cast<const uint8_t *>(&(raw_data[0]));
	size_t data_ptr_idx = 0;

	for(size_t i=0; i < num_vertices; ++i) {
		for(Eigen::Index j=0; j < 3; ++j)
			points[i](j) = GetDataFromBinaryBlob<T>(data_ptr, &data_ptr_idx);

		if(file_contains_normals)
			for(Eigen::Index j=0; j < 3; ++j)
				normals[i](j) = GetDataFromBinaryBlob<T>(data_ptr, &data_ptr_idx);

		if(file_contains_colors) {
			for(size_t j=0; j < 3; ++j)
				colors[i][j] = GetDataFromBinaryBlob<uint8_t>(data_ptr, &data_ptr_idx);
			const uint8_t alpha_color = GetDataFromBinaryBlob<uint8_t>(data_ptr, &data_ptr_idx);

			if(alpha_color != 255)
				return false;
		}

		if(file_contains_intensities)
			intensities[i] = GetDataFromBinaryBlob<T>(data_ptr, &data_ptr_idx);
	}

	if(file_contains_triangles) {
		for(size_t i=0; i < num_triangles; ++i) {
			const uint8_t num_list_elements = GetDataFromBinaryBlob<uint8_t>(data_ptr, &data_ptr_idx);
			if( num_list_elements != 3 )
				return false;
			for(size_t j=0; j < 3; ++j)
				triangles[i][j] = static_cast<size_t>(GetDataFromBinaryBlob<int>(data_ptr, &data_ptr_idx));
		}
	}

	if( output_points != nullptr ) 
		*output_points = points;
	if( output_normals != nullptr ) 
		*output_normals = normals;
	if( output_colors != nullptr ) 
		*output_colors = colors;
	if( output_triangles != nullptr ) 
		*output_triangles = triangles;
	if( output_intensities != nullptr ) 
		*output_intensities = intensities;

	return true;
}

template <typename T>
bool PlyIO<T>::ReadPly(
		const std::string& filename,
		std::vector<geometry::Point<T>>* const output_points,
		std::vector<std::array<size_t, 3> >* const output_triangles
		)
{
	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> points;
	std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1>>> normals;
	std::vector<std::array<uint8_t, 3> > colors;
	std::vector<std::array<size_t, 3> > triangles;
	std::vector<T> intensities;

	const bool ret = ReadPly(filename, &points, &normals, &colors, &triangles, &intensities);

	if( ! ret )
		return false;
	
	output_points->clear();
	output_points->resize(points.size());

	for(size_t i=0; i < points.size(); ++i)
	{
		(*output_points)[i].xyz_(0) = points[i](0);
		(*output_points)[i].xyz_(1) = points[i](1);
		(*output_points)[i].xyz_(2) = points[i](2);
		(*output_points)[i].xyz_(3) = 1.0;
		if( normals.size() != 0 ) 
			(*output_points)[i].n_ = normals[i];
		if( intensities.size() != 0 ) 
			(*output_points)[i].i_ = intensities[i];
		if( colors.size() != 0 ) {
			(*output_points)[i].c_[0] = colors[i][0];
			(*output_points)[i].c_[1] = colors[i][1];
			(*output_points)[i].c_[2] = colors[i][2];
			(*output_points)[i].c_[3] = 1;
		}
	}

	if( output_triangles != nullptr ) *output_triangles = triangles;

	return true;
}



template class PlyIO<float>;
template class PlyIO<double>;

} // namespace ply_io