INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
message(${CSPARSE_INCLUDE_DIR})
SET(G2O_HAVE_OPENGL 0)
message(${G2O_HAVE_OPENGL})
ADD_LIBRARY(tutorial_slam3d_library ${G2O_LIB_TYPE}
edge_pointxyz.cpp       edge_se3_lotsofxyz.h             edge_se3_prior.cpp         parameter_camera.cpp         se2.h                         vertex_pointxyz.h
edge_pointxyz.h         edge_se3_offset.cpp              edge_se3_prior.h           parameter_camera.h           se3_ops.h                     types_slam3d.cpp           vertex_se2.cpp
edge_se2.cpp            edge_se3_offset.h                g2o_tutorial_slam2d_api.h  parameter_se2_offset.cpp     se3_ops.hpp                   types_slam3d.h             vertex_se2.h
edge_se2.h              edge_se3_pointxyz.cpp            g2o_types_slam3d_api.h     parameter_se2_offset.h       se3quat.h                     types_tutorial_slam2d.cpp  vertex_se3.cpp
edge_se2_pointxy.cpp    edge_se3_pointxyz.h              hello.h                    parameter_se3_offset.cpp     simulator.cpp                 types_tutorial_slam2d.h    vertex_se3.h
dquat2mat.cpp                   edge_se2_pointxy.h      edge_se3_pointxyz_depth.cpp      isometry3d_gradients.cpp   parameter_se3_offset.h       simulator.h                   types_tutorial_slam3d.h
dquat2mat.h                     edge_se3.cpp            edge_se3_pointxyz_depth.h        isometry3d_gradients.h     parameter_stereo_camera.cpp  test_isometry3d_mappings.cpp  vertex_point_xy.cpp
dquat2mat.wxm                   edge_se3.h              edge_se3_pointxyz_disparity.cpp  isometry3d_mappings.cpp    parameter_stereo_camera.h        vertex_point_xy.h
#dquat2mat_maxima_generated.cpp
edge_se3_lotsofxyz.cpp  edge_se3_pointxyz_disparity.h    isometry3d_mappings.h      rand.h                           vertex_pointxyz.cpp
)

SET_TARGET_PROPERTIES(tutorial_slam3d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam3d)

TARGET_LINK_LIBRARIES(tutorial_slam3d_library core solver_csparse)

ADD_EXECUTABLE(tutorial_slam3d
  tutorial_slam3d.cpp
)

SET_TARGET_PROPERTIES(tutorial_slam3d PROPERTIES OUTPUT_NAME tutorial_slam3d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(tutorial_slam3d tutorial_slam3d_library)
