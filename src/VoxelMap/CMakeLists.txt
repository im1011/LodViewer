set(VOXMAP_SRC
  VoxelMapAbstract.h
  VoxelMapAveraging.h
  VoxelMapAveraging.cc
)

add_library(voxmap ${VOXMAP_SRC})
target_link_libraries(voxmap
  ${MKL_LIBRARIES}
)

