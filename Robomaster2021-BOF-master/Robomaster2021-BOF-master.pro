# Configs
TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++14
TARGET = Robomaster2021-BOF-master
#Libraries
unix: CONFIG += link_pkgconfig

#OpenCV
unix: PKGCONFIG += opencv


#V4L2
unix:!macx: LIBS += -lpthread
unix:!macx: LIBS += -lv4l2


#Source and header files
HEADERS += \
    General/singleton.hpp \
    General/opencv_extended.h \
    General/numeric_rm.h \
    General/General.h \
    Driver/RMVideoCapture.hpp \
    Serials/Serial.h \
    Main/ImgProdCons.h \
    Pose/AngleSolver.hpp \
    Armor/ArmorDetector.h \
#    Rune/Rune.h \
    Energy/EnergyDetector.h

SOURCES += \
    General/opencv_extended.cpp \
    General/numeric_rm.cpp \
    Driver/RMVideoCapture.cpp \
    Serials/Serial.cpp \
    Main/ImgProdCons.cpp \
    Pose/AngleSolver.cpp \
    Armor/ArmorDetector.cpp \
#    Rune/Rune.cpp \
#   To test a sigular module, uncomment only one of the following
#    Armor/test_armor.cpp
#    Driver/test_camera.cpp
#    Serials/test_serials.cpp
#    Pose/test_angle_2.cpp
#    Pose/test_angle_1.cpp
#   To test a singular module with multi-thread, uncomment main.cpp
#   and one of the others
    Main/main.cpp \
#    Main/test_armor_temp.cpp
#    Main/test_producer.cpp
#    Main/test_armor.cpp
#    Main/test_armor_solver.cpp
#    Main/test_rune.cpp \
#    Main/test_armor_solver_serial.cpp
#     Main/test_recording_video.cpp
    Main/test_infantry.cpp \
    Energy/EnergyDetector.cpp
#    Main/test_sentry.cpp

DISTFILES += \
    Pose/angle_solver_params.xml

