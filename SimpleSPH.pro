#-------------------------------------------------
#
# Project created by QtCreator 2014-11-19T14:51:05
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SimpleSPH
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
    Include/SPHSolver.h \
    Include/PCGSolver.h \
    Include/SparseMatrix.h \
    Include/Simulator.h \
    Include/SPHKernels.h \
    Include/SceneManager.h

SOURCES += \
    Source/Controller.cpp \
    Source/FluidRenderWidget.cpp \
    Source/Main.cpp \
    Source/MainWindow.cpp \
    Source/SPHSolver.cpp \
    Source/Simulator.cpp \
    Source/SceneManager.cpp

RESOURCES += \
    Shader.qrc
