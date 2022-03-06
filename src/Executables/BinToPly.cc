#include <gflags/gflags.h>

#include <FileIO/BinaryIO.h>
#include <FileIO/PlyIO.h>
#include <Geometry/Point.h>

DEFINE_string(bin_file, "", "required");
DEFINE_string(ply_file, "", "required");

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

	binary_io::BinaryReader bin(FLAGS_bin_file);
	std::vector<geometry::Point<float>> all_points_color;
	while(true) {
		geometry::Point<float> p;
		bool ok = true;
		ok = ok && bin.Read<float>(&p.xyz_(0));
		ok = ok && bin.Read<float>(&p.xyz_(1));
		ok = ok && bin.Read<float>(&p.xyz_(2));
		ok = ok && bin.Read<uint8_t>(&p.c_[0]);
		ok = ok && bin.Read<uint8_t>(&p.c_[1]);
		ok = ok && bin.Read<uint8_t>(&p.c_[2]);
		p.c_[3] = 255;
		if(!ok)
			break;
		all_points_color.push_back(p);
	}
	ply_io::PlyIO<float>::WritePly(FLAGS_ply_file, all_points_color);

    return 0;
}