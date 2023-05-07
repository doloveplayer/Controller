cmake_minimum_required(VERSION 3.24)
project(Controller)

set(CMAKE_CXX_STANDARD 11)

include_directories(CommonInc ModernController NonlinearControl TraditionalController)

add_library(Controller library.cpp TraditionalController/Inc/PidController.h TraditionalController/Src/PidController.cpp CommonInc/Common.h CommonInc/Common.cpp)

add_executable(Test Testcode/main.cpp)
target_link_libraries(Test Controller)
target_include_directories(Test PRIVATE
        ${PROJECT_SOURCE_DIR}/CommonInc
        ${PROJECT_SOURCE_DIR}/ModernController
        ${PROJECT_SOURCE_DIR}/NonlinearControl
        ${PROJECT_SOURCE_DIR}/TraditionalController
        )