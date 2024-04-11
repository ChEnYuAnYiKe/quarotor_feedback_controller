cmake_minimum_required(VERSION 3.0.2)
project(quarotor_feedback_controller)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        mavros_msgs
        roscpp
        std_msgs
        nav_msgs
        tf
        )
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES Quarotor_Feedback_Controller
        #  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
        #  DEPENDS system_lib
)
add_library(mpc_solver
        externals/qpoases/SRC/Bounds.cpp
        externals/qpoases/SRC/Constraints.cpp
        externals/qpoases/SRC/CyclingManager.cpp
        externals/qpoases/SRC/Indexlist.cpp
        externals/qpoases/SRC/MessageHandling.cpp
        externals/qpoases/SRC/QProblem.cpp
        externals/qpoases/SRC/QProblemB.cpp
        externals/qpoases/SRC/SubjectTo.cpp
        externals/qpoases/SRC/Utils.cpp
        externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
        model/quadrotor_mpc_codegen/acado_integrator.c
        model/quadrotor_mpc_codegen/acado_solver.c
        model/quadrotor_mpc_codegen/acado_auxiliary_functions.c)
target_include_directories(mpc_solver PUBLIC
        model/quadrotor_mpc_codegen/
        externals/qpoases
        externals/qpoases/INCLUDE
        externals/qpoases/SRC)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#             src/QuarotorFeedbackController.cpp
# )

add_executable(quarotor_feedback_controller1
        src/OffboardNode1.cpp
        src/OffboardWrapper.cpp
        src/QuadrotorFeedbackController.cpp
        src/QuadrotorAggressiveController.cpp
        src/ParamLoad.cpp
        src/MPC/QuadrotorMPCController.cpp
        include/QuadrotorMPCController.h)

add_executable(quarotor_feedback_controller2
        src/OffboardNode2.cpp
        src/OffboardWrapper.cpp
        src/QuadrotorFeedbackController.cpp
        src/QuadrotorAggressiveController.cpp
        src/ParamLoad.cpp
        src/MPC/QuadrotorMPCController.cpp
        include/QuadrotorMPCController.h)

target_link_libraries(quarotor_feedback_controller1 mpc_solver
        ${catkin_LIBRARIES}
        )

target_link_libraries(quarotor_feedback_controller2 mpc_solver
        ${catkin_LIBRARIES}
        )