add_executable(LodViewer LodViewer.cc)
target_link_libraries(LodViewer
	gflags 
	fileio
	gui
)

add_executable(Converter Converter.cc)
target_link_libraries(Converter
	gflags 
	fileio
	voxmap
)

add_executable(BinToPly BinToPly.cc)
target_link_libraries(BinToPly
	gflags 
	fileio
	voxmap
)