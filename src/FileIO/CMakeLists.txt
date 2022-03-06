set(FILEIO_SRC
  PlyIO.h
  PlyIO.cc
  BinaryIO.h
  BinaryIO.cc
  OctreeReader.h
  OctreeReader.cc
)

add_library(fileio ${FILEIO_SRC})
target_link_libraries(fileio
  ${MKL_LIBRARIES}
)
