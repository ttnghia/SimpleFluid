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

#include "SceneManager.h"
#include <QDebug>
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SceneManager::setupScene(Vec_Vec3<float>& particles, Vec_Vec3<float>& velocity)
{
    switch(m_SimParams->scene)
    {
        case SimulationScenes::CubeDrop:
            setupSceneCubeDrop(particles, velocity);
            break;
        case SimulationScenes::SphereDrop:
            setupSceneSphereDrop(particles, velocity);
            break;
        case SimulationScenes::Dambreak:
            setupSceneDambreak(particles, velocity);
            break;
        case SimulationScenes::DoubleDambreak:
            setupSceneDoubleDambreak(particles, velocity);
            break;
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SceneManager::setupSceneCubeDrop(Vec_Vec3<float>& particles, Vec_Vec3<float>& velocity)
{
    Vec3<float> bMin(-0.5f, -0.5f, -0.5f);
    Vec3<float> bMax(0.5f, 0.5f, 0.5f);
    float       spacing = 2.0f * m_SimParams->particleRadius;
    Vec3<int>   grid((bMax[0] - bMin[0]) / spacing,
                     (bMax[1] - bMin[1]) / spacing,
                     (bMax[2] - bMin[2]) / spacing);

    particles.resize(0);
    for(int i = 0; i < grid[0]; ++i)
    {
        for(int j = 0; j < grid[1]; ++j)
        {
            for(int k = 0; k < grid[2]; ++k)
            {
                Vec3<float> ppos = bMin + spacing * Vec3<float>(i, j, k);
                particles.push_back(ppos);
            }
        }
    }

    velocity.assign(particles.size(), Vec3<float>(0));
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SceneManager::setupSceneSphereDrop(Vec_Vec3<float>& particles, Vec_Vec3<float>& velocity)
{
    Vec3<float> center(0.0f, 0.0f, 0.0f);
    float       radius  = 0.5f;
    Vec3<float> bMin    = center - Vec3<float>(radius);
    float       spacing = 2.0f * m_SimParams->particleRadius;
    Vec3<int>   grid    = Vec3<int>(1) * static_cast<int>(2.0f * radius / spacing);

    particles.resize(0);
    for(int i = 0; i < grid[0]; ++i)
    {
        for(int j = 0; j < grid[1]; ++j)
        {
            for(int k = 0; k < grid[2]; ++k)
            {
                Vec3<float> ppos = bMin + spacing * Vec3<float>(i, j, k);
                if(glm::length(ppos - center) > radius)
                    continue;
                particles.push_back(ppos);
            }
        }
    }

    velocity.assign(particles.size(), Vec3<float>(0));
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SceneManager::setupSceneDambreak(Vec_Vec3<float>& particles, Vec_Vec3<float>& velocity)
{
    Vec3<float> bMin(-1.0f, -1.0f, -1.0f);
    Vec3<float> bMax(0.4f, 0.4f, -0.5f);
    bMin += Vec3<float>(m_SimParams->particleRadius);

    float     spacing = 2.0f * m_SimParams->particleRadius;
    Vec3<int> grid((bMax[0] - bMin[0]) / spacing,
                   (bMax[1] - bMin[1]) / spacing,
                   (bMax[2] - bMin[2]) / spacing);

    particles.resize(0);
    for(int i = 0; i < grid[0]; ++i)
    {
        for(int j = 0; j < grid[1]; ++j)
        {
            for(int k = 0; k < grid[2]; ++k)
            {
                Vec3<float> ppos = bMin + spacing * Vec3<float>(i, j, k);
                particles.push_back(ppos);
            }
        }
    }

    velocity.assign(particles.size(), Vec3<float>(0));
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SceneManager::setupSceneDoubleDambreak(Vec_Vec3<float>& particles, Vec_Vec3<float>& velocity)
{
    Vec3<float> bMin;
    Vec3<float> bMax;
    Vec3<int>   grid;
    float       spacing = 2.0f * m_SimParams->particleRadius;
    particles.resize(0);

    ////////////////////////////////////////////////////////////////////////////////
    // first block
    bMin = Vec3<float>(-1.0f, -1.0f, -1.0f) + Vec3<float>(m_SimParams->particleRadius);
    bMax = Vec3<float>(0.4f, 0.4f, -0.5f);
    grid = Vec3<int>((bMax[0] - bMin[0]) / spacing,
                     (bMax[1] - bMin[1]) / spacing,
                     (bMax[2] - bMin[2]) / spacing);

    for(int i = 0; i < grid[0]; ++i)
    {
        for(int j = 0; j < grid[1]; ++j)
        {
            for(int k = 0; k < grid[2]; ++k)
            {
                Vec3<float> ppos = bMin + spacing * Vec3<float>(i, j, k);
                particles.push_back(ppos);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // second block
    bMin  = Vec3<float>(-0.4f, -1.0f, 0.5f);
    bMin += Vec3<float>(0, m_SimParams->particleRadius, 0);
    bMax  = Vec3<float>(1.0f, 0.4f, 1.0f) - Vec3<float>(m_SimParams->particleRadius, 0, m_SimParams->particleRadius);
    grid  = Vec3<int>((bMax[0] - bMin[0]) / spacing,
                      (bMax[1] - bMin[1]) / spacing,
                      (bMax[2] - bMin[2]) / spacing);

    for(int i = 0; i < grid[0]; ++i)
    {
        for(int j = 0; j < grid[1]; ++j)
        {
            for(int k = 0; k < grid[2]; ++k)
            {
                Vec3<float> ppos = bMax - spacing * Vec3<float>(i, j, k);
                particles.push_back(ppos);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    velocity.assign(particles.size(), Vec3<float>(0));
}
