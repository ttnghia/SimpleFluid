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

#include <Banana/Timer.h>
#include <Banana/Macros.h>

#include <QObject>
#include <QStringList>

#include <future>

#include <tbb/tbb.h>

#include "Common.h"
#include "QtSPHSolver.h"
#include "SceneManager.h"

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class Simulator : public QObject
{
    Q_OBJECT

public:
    Simulator(const std::shared_ptr<ParticleSystemData>& particleData) : m_ParticleData(particleData)
    {
        m_SPHSolver    = std::make_unique<QtSPHSolver>(m_ParticleData, m_SimParams);
        m_SceneManager = std::make_unique<SceneManager>(m_SimParams);
    }

    const std::shared_ptr<SPHParameters<float> >& getSimParams() { return m_SimParams; }

    bool isRunning() { return !m_bStop; }
    void stop();
    void reset();
    void startSimulation();

public slots:
    void doSimulation();
    void changeScene(SimulationScenes::Scene scene);
    void setupScene();

signals:
    void simulationFinished();
    void systemTimeChanged(float time);
    void numParticleChanged(unsigned int numParticles);
    void particleChanged();
    void frameFinished();

protected:
    std::unique_ptr<tbb::task_scheduler_init> m_ThreadInit   = nullptr;
    volatile bool                             m_bStop        = true;
    std::shared_ptr<ParticleSystemData>       m_ParticleData = nullptr;

    float                                  m_SimTime      = 0;
    std::unique_ptr<SceneManager>          m_SceneManager = nullptr;
    std::unique_ptr<QtSPHSolver>           m_SPHSolver    = nullptr;
    std::shared_ptr<SPHParameters<float> > m_SimParams    = std::make_shared<SPHParameters<float> >();
    std::future<void>                      m_SimulationFutureObj;
};