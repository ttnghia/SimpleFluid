//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  Copyright (c) 2017 by
//       __      _     _         _____
//    /\ \ \__ _| |__ (_) __ _  /__   \_ __ _   _  ___  _ __   __ _
//   /  \/ / _` | '_ \| |/ _` |   / /\/ '__| | | |/ _ \| '_ \ / _` |
//  / /\  / (_| | | | | | (_| |  / /  | |  | |_| | (_) | | | | (_| |
//  \_\ \/ \__, |_| |_|_|\__,_|  \/   |_|   \__,_|\___/|_| |_|\__, |
//         |___/                                              |___/
//
//  <nghiatruong.vn@gmail.com>
//  All rights reserved.
//
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

#pragma once

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#define DEFAULT_CAMERA_POSITION glm::vec3(-3.0, 0.8, 0.0)
#define DEFAULT_CAMERA_FOCUS    glm::vec3(0, -0.2, 0)

#define CUSTOM_PARTICLE_MATERIAL         \
    {                                    \
        glm::vec4(0.2 * 0.2),            \
        glm::vec4(1.0, 0.63, 0.3, 1.00), \
        glm::vec4(1),                    \
        250.0,                           \
        std::string("ParticleMaterial")  \
    }


#define PARTICLE_COLOR_RAMP       \
    {                             \
        glm::vec3(1.0, 0.0, 0.0), \
        glm::vec3(1.0, 0.5, 0.0), \
        glm::vec3(1.0, 1.0, 0.0), \
        glm::vec3(1.0, 0.0, 1.0), \
        glm::vec3(0.0, 1.0, 0.0), \
        glm::vec3(0.0, 1.0, 1.0), \
        glm::vec3(0.0, 0.0, 1.0)  \
    }


#define DEFAULT_FLOOR_SIZE 10

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class SimulationScenes
{
public:
    enum Scene
    {
        SphereDrop=0,
        CubeDrop,
        Dambreak,
        DoubleDambreak
    };
};
#define SimulationSceneNames { QString("SphereDrop"), QString("CubeDrop"), QString("Dambreak"), QString("DoubleDambreak") }

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class ParticleColorMode
{
public:
    enum
    {
        Uniform = 0,
        Random,
        Ramp,
        NumColorMode
    };
};

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#include <QStringList>
#include <QString>
#include <QDir>
////////////////////////////////////////////////////////////////////////////////
inline QStringList getTextureFolders(QString texType)
{
    QDir dataDir(QDir::currentPath() + "/Textures/" + texType);
    dataDir.setFilter(QDir::NoDotAndDotDot | QDir::Dirs);

    return dataDir.entryList();
}

inline QStringList getTextureFiles(QString texType)
{
    QDir dataDir(QDir::currentPath() + "/Textures/" + texType);
    dataDir.setFilter(QDir::NoDotAndDotDot | QDir::Files);

    return dataDir.entryList();
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#include <Banana/TypeNames.h>
#include <Banana/Data/ParticleSystemData.h>
#include <memory>
#include <QVector3D>
#include <QDebug>
using namespace Banana;