set(VIEW_BASE_SRC
  ShaderWrapper.h
  ShaderWrapper.cc
  ViewBase.h
)

add_library(gui_view_base ${VIEW_BASE_SRC})
target_link_libraries(gui_view_base
  ${MKL_LIBRARIES}
  ${QT_LIBS}
)

add_subdirectory(PointCloudView)
add_subdirectory(OctreeView)
