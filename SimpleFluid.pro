#-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#
#  Copyright (c) 2017 by
#       __      _     _         _____
#    /\ \ \__ _| |__ (_) __ _  /__   \_ __ _   _  ___  _ __   __ _
#   /  \/ / _` | '_ \| |/ _` |   / /\/ '__| | | |/ _ \| '_ \ / _` |
#  / /\  / (_| | | | | | (_| |  / /  | |  | |_| | (_) | | | | (_| |
#  \_\ \/ \__, |_| |_|_|\__,_|  \/   |_|   \__,_|\___/|_| |_|\__, |
#         |___/                                              |___/
#
#  <nghiatruong.vn@gmail.com>
#  All rights reserved.
#
#-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SimpleFluid
TEMPLATE = app

INCLUDEPATH += $$PWD/Include

include (../../Banana/BananaCore/BananaCore.pri)
include (../../Banana/QtAppHelpers/QtAppHelpers.pri)
include (../../Banana/OpenGLHelpers/OpenGLHelpers.pri)
include (../../Banana/Simulation/Simulation.pri)

HEADERS += \
    Include/Controller.h \
    Include/FluidRenderWidget.h \
    Include/MainWindow.h \
    Include/Common.h \
    Include/Simulator.h \
    Include/SceneManager.h \
    Include/QtSPHSolver.h

SOURCES += \
    Source/Controller.cpp \
    Source/FluidRenderWidget.cpp \
    Source/Main.cpp \
    Source/MainWindow.cpp \
    Source/Simulator.cpp \
    Source/SceneManager.cpp

RESOURCES += \
    Shader.qrc
