set(LIDAR_POINT_CLOUD_RGB_VIEW_SRC
  PointCloudView.h
  PointCloudView.cc
)

add_library(gui_pointcloud_view ${LIDAR_POINT_CLOUD_RGB_VIEW_SRC})
target_link_libraries(gui_pointcloud_view
  ${MKL_LIBRARIES}
  ${QT_LIBS}
  gui_view_base
)
