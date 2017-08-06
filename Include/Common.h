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
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// SimulationParameters
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
#include <Banana/TypeNames.h>
#include <Banana/Data/ParticleSystemData.h>
#include <memory>
#include <QVector3D>
#include <QDebug>

enum class FLIPInterpolationKernel
{
    LinearKernel,
    CubicSplineKernel
};

#define DEFAULT_RESOLUTION           24

#define DEFAULT_BOUNDARY_RESTITUTION 0.1f
#define DEFAULT_PRESSURE_STIFFNESS   50000.0f
#define DEFAULT_NEAR_FORCE_STIFFNESS 50000.0f
#define DEFAULT_VISCOSITY            0.05

class SimulationParameters
{
public:
    SimulationParameters() { updateParams(); }

    SimulationScenes::Scene scene = SimulationScenes::SphereDrop;

    int   numThreads      = 0;
    float stopTime        = 5.0;
    float defaultTimestep = 1.0e-4;

    Vec3<float> boxMin = Vec3<float>(-1.0f, -1.0f, -1.0f);
    Vec3<float> boxMax = Vec3<float>(1.0f, 1.0f, 1.0f);

    float pressureStiffness  = DEFAULT_PRESSURE_STIFFNESS;
    float nearForceStiffness = DEFAULT_NEAR_FORCE_STIFFNESS;
    float viscosity          = DEFAULT_VISCOSITY;
    float kernelRadius       = 2.0f / float(DEFAULT_RESOLUTION);

    bool bCorrectDensity        = false;
    bool bUseBoundaryParticles  = true;
    bool bUseAttractivePressure = false;

    float boundaryRestitution     = DEFAULT_BOUNDARY_RESTITUTION;
    float attractivePressureRatio = 0.1f;
    float restDensity             = 1000.0f;

    // the following need to be computed
    float particleMass;
    float particleRadius;
    float kernelRadiusSqr;
    float nearKernelRadius;
    float restDensitySqr;

    void updateParams()
    {
        particleRadius   = kernelRadius / 4.0f;
        kernelRadiusSqr  = kernelRadius * kernelRadius;
        nearKernelRadius = particleRadius * 2.5f;

        particleMass   = pow(2.0 * particleRadius, 3) * restDensity * 0.9;
        restDensitySqr = restDensity * restDensity;
    }
};

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class FluidSolver
{
public:
    FluidSolver(const std::shared_ptr<SimulationParameters>& simParams) : m_SimParams(simParams) {}

    virtual float advanceFrame() = 0;
    virtual void  makeReady()    = 0;

    virtual unsigned int     getNumParticles() = 0;
    virtual Vec_Vec3<float>& getParticles()    = 0;
    virtual Vec_Vec3<float>& getVelocity()     = 0;

protected:
    std::shared_ptr<SimulationParameters> m_SimParams;
};