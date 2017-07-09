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
#include <Banana/Timer.h>
#include <Banana/Macros.h>

#include <QObject>
#include <QStringList>

#include <tbb/tbb.h>

#include "Common.h"
#include "SPHSolver.h"

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class Simulator : public QObject
{
    Q_OBJECT
public:
    Simulator() : m_ParticleData(nullptr) {}
    Simulator(std::shared_ptr<ParticleSystemData>& particleData) : m_ParticleData(particleData) {}

    void                                       setParticleData(const std::shared_ptr<ParticleSystemData>& particleData);
    const std::shared_ptr<ParticleSystemData>& getParticleDataObj();

    void startSimulation(const std::shared_ptr<SimulationParameters>& simParams);
    void doSimulation();
    void pause();
    void reset();

public slots:
    void createScene();

signals:
    void frameFinished();
    void numParticleChanged(unsigned int numParticles);
    void currentFrameChanged(int currentFrame);
    void logChanged(const QString& logStr);
    void simInfoChanged(const QStringList& simInfo);

protected:
    bool                                  m_bPause    = false;
    bool                                  m_bStop     = false;
    std::shared_ptr<SimulationParameters> m_SimParams = nullptr;
    std::shared_ptr<ParticleSystemData>   m_ParticleData;
    Timer                                 m_Timer;

    std::unique_ptr<SPHSolver> m_SPHSolver = nullptr;
};