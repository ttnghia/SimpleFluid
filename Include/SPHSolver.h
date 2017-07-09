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

#include <Banana/TypeNames.h>
#include <Banana/Array/Array3.h>
#include <Grid/Grid3D.h>

#include "Common.h"
#include "SPHKernels.h"

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class SPHSolver
{
public:
    SPHSolver(std::shared_ptr<ParticleSystemData>& particleData, const SimulationParameters& simParams) :
        m_SimData(particleData), m_SimParams(simParams)
    {
        m_CubicKernel.setRadius(simParams.kernelRadius);
        m_SpikyKernel.setRadius(simParams.kernelRadius);
        m_NearSpikyKernel.setRadius(1.5 * simParams.particleRadius);
        m_Grid3D.setCellSize(simParams.kernelRadius);

        generateBoundaryParticles();
    }

    void makeReady();

private:
    void  advanceVelocity(float timeStep);
    void  moveParticles(float dt);
    float computeMaxVel();
    float computeCFLtimeStep();

    void generateBoundaryParticles();
    void computeDensity();
    void correctDensity();
    void computeRepulsiveVelocity(float timeStep);
    void addGravity(float timeStep);
    void computePressureAccelerations();
    void updateVelocity(float timeStep);
    void computeViscosity();

    ////////////////////////////////////////////////////////////////////////////////
    struct SimData
    {
        SimData(std::shared_ptr<ParticleSystemData>& particleData) : particles(*particleData->getArray("Position")) {}

        Array3_VecUInt   cellParticles;
        Vec_Vec3<float>& particles;
        Vec_Vec3<float>  velocity;

        Vec_Vec3<float> bd_particles_lx;
        Vec_Vec3<float> bd_particles_ux;
        Vec_Vec3<float> bd_particles_ly;
        Vec_Vec3<float> bd_particles_uy;
        Vec_Vec3<float> bd_particles_lz;
        Vec_Vec3<float> bd_particles_uz;

        Vec_Vec3<float>    pressureAcc;
        std::vector<float> density;
    } m_SimData;

    PrecomputedKernel<CubicKernel, 10000> m_CubicKernel;
    PrecomputedKernel<SpikyKernel, 10000> m_SpikyKernel;
    PrecomputedKernel<SpikyKernel, 10000> m_NearSpikyKernel;
    Grid3D<float>                         m_Grid3D;

    ////////////////////////////////////////////////////////////////////////////////
    const SimulationParameters& m_SimParams;
};


