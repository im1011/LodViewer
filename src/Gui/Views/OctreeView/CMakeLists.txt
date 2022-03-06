set(OCTREE_VIEW_SRC
  OctreeView.h
  OctreeView.cc
)

add_library(gui_octree_view ${OCTREE_VIEW_SRC})
target_link_libraries(gui_octree_view
  ${MKL_LIBRARIES}
  ${QT_LIBS}
  gui_view_base
  gui_pointcloud_view
  fileio
)
