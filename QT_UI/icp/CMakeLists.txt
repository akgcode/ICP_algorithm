cmake_minimum_required(VERSION 3.5)

project(icp LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

#find_package(Qt5 COMPONENTS Widgets LinguistTools REQUIRED NO_DEFAULT_PATH)
find_package(Qt5 COMPONENTS Widgets LinguistTools REQUIRED)
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_executable(runfile run_icp.cpp)
#add_executable(runfile show_icp.cpp) #use any one add_executable at a time
#target_link_libraries (runfile ${PCL_LIBRARIES})
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

if(ANDROID)
  add_library(icp SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
else()
  add_executable(icp

    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
endif()

target_link_libraries(icp PRIVATE Qt5::Widgets ${PCL_LIBRARIES})
