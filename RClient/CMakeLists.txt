cmake_minimum_required(VERSION 3.16)

project(RClient VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(Qt6 REQUIRED COMPONENTS Network)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(RClient
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        ../Common/ControlPanelProtocol.h
        ../Common/ControlPanelProtocol.cpp
	FileUtils.cpp InputHandling.cpp main.cpp mainwindow.cpp MapWidget.cpp SensorDataFileWriter.cpp SensorScanWidget.cpp UIEventsHandling.cpp FileUtils.h InputHandling.h mainwindow.h MapWidget.h SensorDataFileWriter.h SensorScanWidget.h
	../Common/Map.cpp ../Common/Map.h
	../Common/Common.h ../Common/ControlInputsProtocol.h ../Common/MapUtils.cpp ../Common/MapUtils.h ../Common/ProbUtils.cpp ../Common/ProbUtils.h ../Common/Sensor.cpp ../Common/Sensor.h
	../Common/Chassis.h ../Common/ControlPanelProtocol.h ../Common/Sensor.cpp ../Common/Sensor.h
	../Common/StringWordWriter.cpp ../Common/StringWordWriter.h
	../Common/StringWordReader.cpp ../Common/StringWordReader.h
    )

# Define target properties for Android with Qt 6 as:
#    set_property(TARGET RClient APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(RClient SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(RClient
            ${PROJECT_SOURCES}
        )
    endif()
endif()

qt_standard_project_setup(REQUIRES 6.5)

target_link_libraries(RClient PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(RClient PRIVATE Qt6::Network)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.RClient)
endif()
set_target_properties(RClient PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS RClient
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(RClient)
endif()
