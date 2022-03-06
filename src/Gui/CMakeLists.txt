set(GUI_SRC
  Window.h
  Window.cc
  OpenGlWidget.h
  OpenGlWidget.cc
)

add_library(gui ${GUI_SRC})
target_link_libraries(gui
  ${MKL_LIBRARIES}
  ${QT_LIBS}
  gui_pointcloud_view
  gui_octree_view
  fileio
)

add_subdirectory(Views)
