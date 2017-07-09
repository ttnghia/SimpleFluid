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

#include "SPHSolver.h"

#include <Banana/ParallelHelpers/ParallelObjects.h>

#include <tbb/tbb.h>
#include <random>

#define CLAMP_THRESHOLD_DENSITY_RATIO_MAX 10.0
#define CLAMP_THRESHOLD_DENSITY_RATIO_MIN 0.1

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::makeReady()
{
    if(m_SimData.velocity.size() != m_SimData.particles.size())
    {
        m_SimData.velocity.resize(m_SimData.particles.size());
        m_SimData.velocity.assign(m_SimData.velocity.size(), Vec3<float>(0, 0, 0));
    }

    m_SimData.pressureAcc.resize(m_SimData.particles.size());
    m_SimData.density.resize(m_SimData.particles.size());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float SPHSolver::computeMaxVel()
{
    static std::vector<float> mag2_velocity;
    mag2_velocity.resize(m_SimData.velocity.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, mag2_velocity.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              mag2_velocity[p] = glm::length2(m_SimData.velocity[p]);
                          }
                      }); // end parallel_for

    // then, find the maximum value
    ParallelObjects::VectorMaxElement<float> m2v(mag2_velocity);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, mag2_velocity.size()), m2v);

    return sqrt(m2v.getResult());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float SPHSolver::computeCFLtimeStep()
{
    const float cflFactor = 0.5f;

    float maxVel      = computeMaxVel();
    float CFLtimeStep = maxVel > 1e-8 ? cflFactor * 0.4f * (2.0f * m_SimParams.particleRadius / maxVel) : 1e10;

    CFLtimeStep = fmax(CFLtimeStep, m_SimParams.defaulttimeStep * 0.1f);
    CFLtimeStep = fmin(CFLtimeStep, m_SimParams.defaulttimeStep * 10.0f);

    return CFLtimeStep;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::generateBoundaryParticles()
{
    m_SimData.bd_particles_lx.clear();
    m_SimData.bd_particles_ux.clear();
    m_SimData.bd_particles_ly.clear();
    m_SimData.bd_particles_uy.clear();
    m_SimData.bd_particles_lz.clear();
    m_SimData.bd_particles_uz.clear();

    std::random_device                    rd;
    std::mt19937                          gen(rd());
    std::uniform_real_distribution<float> dis01(0, 0.1 * m_SimParams.particleRadius);
    std::uniform_real_distribution<float> dis13(0.1 * m_SimParams.particleRadius, 0.3 * m_SimParams.particleRadius);

    const float spacing_ratio = 1.7;
    const float spacing       = m_SimParams.particleRadius * spacing_ratio;

    const int         num_particles_1d = 1 + static_cast<int>(ceil((2 + 1) * m_SimParams.kernelRadius / spacing));
    const int         num_layers       = static_cast<int>(ceil(m_SimParams.kernelRadius / spacing));
    const Vec2<float> corner           = Vec2<float>(1, 1) * (-m_SimParams.kernelRadius + m_SimParams.particleRadius);

    for(int l1 = 0; l1 < num_particles_1d; ++l1)
    {
        for(int l2 = 0; l2 < num_particles_1d; ++l2)
        {
            const Vec2<float> plane_pos = corner + Vec2<float>(l1, l2) * spacing;

            for(int l3 = 0; l3 < num_layers; ++l3)
            {
                const float layer_pos = m_SimParams.particleRadius + spacing * l3;

                Vec3<float> pos_lx = Vec3<float>(m_SimParams.boxMin[0] - layer_pos, plane_pos[0], plane_pos[1]) + Vec3<float>(dis01(gen), dis13(gen), dis13(gen));
                Vec3<float> pos_ux = Vec3<float>(m_SimParams.boxMax[0] + layer_pos, plane_pos[0], plane_pos[1]) + Vec3<float>(dis01(gen), dis13(gen), dis13(gen));
                Vec3<float> pos_ly = Vec3<float>(plane_pos[0], m_SimParams.boxMin[1] - layer_pos, plane_pos[1]) + Vec3<float>(dis13(gen), dis01(gen), dis13(gen));
                Vec3<float> pos_uy = Vec3<float>(plane_pos[0], m_SimParams.boxMax[1] + layer_pos, plane_pos[1]) + Vec3<float>(dis13(gen), dis01(gen), dis13(gen));
                Vec3<float> pos_lz = Vec3<float>(plane_pos[0], plane_pos[1], m_SimParams.boxMin[2] - layer_pos) + Vec3<float>(dis13(gen), dis13(gen), dis01(gen));
                Vec3<float> pos_uz = Vec3<float>(plane_pos[0], plane_pos[1], m_SimParams.boxMax[2] + layer_pos) + Vec3<float>(dis13(gen), dis13(gen), dis01(gen));

                m_SimData.bd_particles_lx.push_back(pos_lx);
                m_SimData.bd_particles_ux.push_back(pos_ux);

                m_SimData.m_SimData.bd_particles_ly.push_back(pos_ly);
                m_SimData.m_SimData.bd_particles_uy.push_back(pos_uy);

                m_SimData.m_SimData.bd_particles_lz.push_back(pos_lz);
                m_SimData.m_SimData.bd_particles_uz.push_back(pos_uz);
            }
        }
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::advanceVelocity(float timeStep)
{
    computeDensity();

    if(m_SimParams.bCorrectDensity)
    {
        correctDensity();
    }

    if(m_SimParams.bUseRepulsiveForce)
    {
        computeRepulsiveVelocity(timeStep);
    }

    addGravity(timeStep);
    computePressureAccelerations();
    updateVelocity(timeStep);
    computeViscosity();
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::moveParticles(float dt)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_SimData.particles.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              Vec3<float> pvel = m_SimData.velocity[p];
                              Vec3<float> ppos = m_SimData.particles[p] + pvel * dt;

                              bool velChanged = false;

                              for(int l = 0; l < 3; ++l)
                              {
                                  if(ppos[l] < m_SimParams.boxMin[l])
                                  {
                                      ppos[l] = m_SimParams.boxMin[l];
                                      pvel[l] *= -m_SimParams.boundaryRestitution;
                                      velChanged = true;
                                  }
                                  else if(ppos[l] > m_SimParams.boxMax[l])
                                  {
                                      ppos[l] = m_SimParams.boxMax[l];
                                      pvel[l] *= -m_SimParams.boundaryRestitution;
                                      velChanged = true;
                                  }
                              }

                              m_SimData.particles[p] = ppos;
                              if(velChanged)
                                  m_SimData.velocity[p] = pvel;
                          }
                      }); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::computeDensity()
{
    static const float valid_lx    = m_SimParams.boxMin[0] + m_SimParams.kernelRadius;
    static const float valid_ux    = m_SimParams.boxMax[0] - m_SimParams.kernelRadius;
    static const float valid_ly    = m_SimParams.boxMin[1] + m_SimParams.kernelRadius;
    static const float valid_uy    = m_SimParams.boxMax[1] - m_SimParams.kernelRadius;
    static const float valid_lz    = m_SimParams.boxMin[2] + m_SimParams.kernelRadius;
    static const float valid_uz    = m_SimParams.boxMax[2] - m_SimParams.kernelRadius;
    static const float min_density = m_SimParams.restDensity * CLAMP_THRESHOLD_DENSITY_RATIO_MIN;
    static const float max_density = m_SimParams.restDensity * CLAMP_THRESHOLD_DENSITY_RATIO_MAX;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_SimData.particles.size()), [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              const Vec3& ppos = m_SimData.particles[p];
                              const Vec3i& pcellId = m_Grid3D.getCellIdx<int>(ppos);
                              float pden = m_CubicKernel.W_zero();

                              for(int lk = -1; lk <= 1; ++lk)
                              {
                                  for(int lj = -1; lj <= 1; ++lj)
                                  {
                                      for(int li = -1; li <= 1; ++li)
                                      {
                                          const Vec3i cellId = pcellId + Vec3i(li, lj, lk);

                                          if(!m_Grid3D.isValidCell<int>(cellId))
                                          {
                                              continue;
                                          }

                                          for(UInt32 q : m_SimData.cellParticles(cellId))
                                          {
                                              if((UInt32)p == q)
                                              {
                                                  continue;
                                              }

                                              const Vec3& qpos = m_SimData.particles[q];
                                              const Vec3<float> r = qpos - ppos;

                                              pden += m_CubicKernel.W(r);
                                          }
                                      }
                                  }
                              } // end loop over neighbor cells


                              ////////////////////////////////////////////////////////////////////////////////
                              // ==> correct density for the boundary particles
                              if(m_SimParams.bUseBoundaryParticles)
                              {
                                  // => lx/ux
                                  if(ppos[0] < valid_lx || ppos[0] > valid_ux)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(0,
                                                                                                                    floor(ppos[1] / m_SimParams.kernelRadius),
                                                                                                                    floor(ppos[2] / m_SimParams.kernelRadius));

                                      const Vec_Vec3& bparticles = (ppos[0] < valid_lx) ? m_SimData.bd_particles_lx : m_SimData.bd_particles_ux;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          pden += m_CubicKernel.W(r);
                                      }
                                  }


                                  // => ly/uy
                                  if(ppos[1] < valid_ly || ppos[1] > valid_uy)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(floor(ppos[0] / m_SimParams.kernelRadius),
                                                                                                                    0,
                                                                                                                    floor(ppos[2] / m_SimParams.kernelRadius));

                                      const Vec_Vec3& bparticles = (ppos[1] < valid_ly) ? m_SimData.bd_particles_ly : m_SimData.bd_particles_uy;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          pden += m_CubicKernel.W(r);
                                      }
                                  }


                                  // => lz/uz
                                  if(ppos[2] < valid_lz || ppos[2] > valid_uz)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(floor(ppos[0] / m_SimParams.kernelRadius),
                                                                                                                    floor(ppos[1] / m_SimParams.kernelRadius),
                                                                                                                    0);


                                      const Vec_Vec3& bparticles = (ppos[2] < valid_lz) ? m_SimData.bd_particles_lz : m_SimData.bd_particles_uz;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          pden += m_CubicKernel.W(r);
                                      }
                                  }
                              }

                              // <= end boundary density correction
                              ////////////////////////////////////////////////////////////////////////////////


                              m_SimData.density[p] = pden < 1.0 ? 0 : fmin(fmax(pden, min_density), max_density);
                          }
                      }); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::correctDensity()
{
    static const float valid_lx    = m_SimParams.boxMin[0] + m_SimParams.kernelRadius;
    static const float valid_ux    = m_SimParams.boxMax[0] - m_SimParams.kernelRadius;
    static const float valid_ly    = m_SimParams.boxMin[1] + m_SimParams.kernelRadius;
    static const float valid_uy    = m_SimParams.boxMax[1] - m_SimParams.kernelRadius;
    static const float valid_lz    = m_SimParams.boxMin[2] + m_SimParams.kernelRadius;
    static const float valid_uz    = m_SimParams.boxMax[2] - m_SimParams.kernelRadius;
    static const float min_density = m_SimParams.restDensity * CLAMP_THRESHOLD_DENSITY_RATIO_MIN;
    static const float max_density = m_SimParams.restDensity * CLAMP_THRESHOLD_DENSITY_RATIO_MAX;

    static std::vector<float> tmp_density;
    tmp_density.resize(m_SimData.density.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_SimData.particles.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              const Vec3& ppos = m_SimData.particles[p];
                              const Vec3i& pcellId = m_Grid3D.getCellIdx<int>(ppos);
                              float tmp = m_CubicKernel.W_zero() / density[p];

                              for(int lk = -1; lk <= 1; ++lk)
                              {
                                  for(int lj = -1; lj <= 1; ++lj)
                                  {
                                      for(int li = -1; li <= 1; ++li)
                                      {
                                          const Vec3i cellId = pcellId + Vec3i(li, lj, lk);

                                          if(!m_Grid3D.isValidCell<int>(cellId))
                                          {
                                              continue;
                                          }

                                          for(UInt32 q : m_SimData.cellParticles(cellId))
                                          {
                                              if((UInt32)p == q)
                                              {
                                                  continue;
                                              }

                                              const Vec3& qpos = m_SimData.particles[q];
                                              const Vec3<float> r = qpos - ppos;
                                              const float qden = m_SimData.density[q];

                                              if(qden < 1e-8)
                                              {
                                                  continue;
                                              }

                                              tmp += m_CubicKernel.W(r) / qden;
                                          }
                                      }
                                  }
                              } // end loop over neighbor cells



                              ////////////////////////////////////////////////////////////////////////////////
                              // ==> correct density for the boundary particles
                              if(m_SimParams.bUseBoundaryParticles)
                              {
                                  // => lx/ux
                                  if(ppos[0] < valid_lx || ppos[0] > valid_ux)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(0,
                                                                                                                    floor(ppos[1] / m_SimParams.kernelRadius),
                                                                                                                    floor(ppos[2] / m_SimParams.kernelRadius));

                                      const Vec_Vec3& bparticles = (ppos[0] < valid_lx) ? m_SimData.bd_particles_lx : m_SimData.bd_particles_ux;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;

                                          tmp += m_CubicKernel.W(r) / m_SimParams.restDensity;
                                      }
                                  }


                                  // => ly/uy
                                  if(ppos[1] < valid_ly || ppos[1] > valid_uy)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(floor(ppos[0] / m_SimParams.kernelRadius),
                                                                                                                    0,
                                                                                                                    floor(ppos[2] / m_SimParams.kernelRadius));

                                      const Vec_Vec3& bparticles = (ppos[1] < valid_ly) ? m_SimData.bd_particles_ly : m_SimData.bd_particles_uy;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          tmp += m_CubicKernel.W(r) / m_SimParams.restDensity;
                                      }
                                  }


                                  // => lz/uz
                                  if(ppos[2] < valid_lz || ppos[2] > valid_uz)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(floor(ppos[0] / m_SimParams.kernelRadius),
                                                                                                                    floor(ppos[1] / m_SimParams.kernelRadius),
                                                                                                                    0);


                                      const Vec_Vec3& bparticles = (ppos[2] < valid_lz) ? m_SimData.bd_particles_lz : m_SimData.bd_particles_uz;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;

                                          tmp += m_CubicKernel.W(r) / m_SimParams.restDensity;
                                      }
                                  }
                              }

                              // <= end boundary density correction
                              ////////////////////////////////////////////////////////////////////////////////

                              tmp_density[p] = tmp > 1e-8 ? m_SimData.density[p] / fmin(tmp, max_density) : 0;
                          }
                      }); // end parallel_for

    std::copy(tmp_density.begin(), tmp_density.end(), m_SimData.density.begin());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::computeRepulsiveVelocity(float timeStep)
{}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::addGravity(float timeStep)
{}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::computePressureAccelerations()
{
    static const float valid_lx = m_SimParams.boxMin[0] + m_SimParams.kernelRadius;
    static const float valid_ux = m_SimParams.boxMax[0] - m_SimParams.kernelRadius;
    static const float valid_ly = m_SimParams.boxMin[1] + m_SimParams.kernelRadius;
    static const float valid_uy = m_SimParams.boxMax[1] - m_SimParams.kernelRadius;
    static const float valid_lz = m_SimParams.boxMin[2] + m_SimParams.kernelRadius;
    static const float valid_uz = m_SimParams.boxMax[2] - m_SimParams.kernelRadius;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, particles.size()), [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              const float pden = m_SimData.density[p];

                              //Vec3<float> pforce(0, 0, 0);
                              Vec3<float> pressure_accel(0, 0, 0);

                              if(pden < 1e-8)
                              {
                                  m_SimData.pressureAcc[p] = pressure_accel;
                                  continue;
                              }

                              const float pdrho = POW7(pden / m_SimParams.restDensity) - 1.0;
                              const float ppressure = m_SimParams.bUseAttractivePressure ? fmax(pdrho, pdrho * m_SimParams.attractive_repulsive_pressure_ratio) : fmax(pdrho, 0);

                              const Vec3& ppos = particles[p];
                              const Vec3i pcellId = m_Grid3D.getCellIdx<int>(ppos);

                              for(int lk = -1; lk <= 1; ++lk)
                              {
                                  for(int lj = -1; lj <= 1; ++lj)
                                  {
                                      for(int li = -1; li <= 1; ++li)
                                      {
                                          const Vec3i cellId = pcellId + Vec3i(li, lj, lk);

                                          if(!m_Grid3D.isValidCell<int>(cellId))
                                          {
                                              continue;
                                          }

                                          for(UInt32 q : m_SimData.cellParticles(cellId))
                                          {
                                              if((UInt32)p == q)
                                              {
                                                  continue;
                                              }

                                              const Vec3& qpos = m_SimData.particles[q];
                                              const float qden = m_SimData.density[q];

                                              const Vec3<float> r = qpos - ppos;
                                              if(glm::length2(r) > m_SimParams.kernelRadiusSqr)
                                              {
                                                  continue;
                                              }

                                              if(qden < 1e-8)
                                              {
                                                  continue;
                                              }

                                              // pressure force
                                              const float qdrho = POW7(qden / m_SimParams.restDensity) - 1.0;
                                              const float qpressure = m_SimParams.bUseAttractivePressure ? fmax(qdrho, qdrho * m_SimParams.attractive_repulsive_pressure_ratio) : fmax(qdrho, 0);


                                              const Vec3<float> pressure = (ppressure / (pden * pden) + qpressure / (qden * qden)) * m_SpikyKernel.gradW(r);
                                              pressure_accel += pressure;
                                          }
                                      }
                                  }
                              } // end loop over neighbor cells

                              ////////////////////////////////////////////////////////////////////////////////
                              // ==> correct density for the boundary particles
                              if(m_SimParams.bUseBoundaryParticles)
                              {
                                  // => lx/ux
                                  if(ppos[0] < valid_lx || ppos[0] > valid_ux)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(0,
                                                                                                                    floor(ppos[1] / m_SimParams.kernelRadius),
                                                                                                                    floor(ppos[2] / m_SimParams.kernelRadius));

                                      const Vec_Vec3& bparticles = (ppos[0] < valid_lx) ? m_SimData.bd_particles_lx : m_SimData.bd_particles_ux;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          const Vec3<float> pressure = (ppressure / (pden * pden)) * m_SpikyKernel.gradW(r);
                                          pressure_accel += pressure;
                                      }
                                  }


                                  // => ly/uy
                                  if(ppos[1] < valid_ly || ppos[1] > valid_uy)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(floor(ppos[0] / m_SimParams.kernelRadius),
                                                                                                                    0,
                                                                                                                    floor(ppos[2] / m_SimParams.kernelRadius));

                                      const Vec_Vec3& bparticles = (ppos[1] < valid_ly) ? m_SimData.bd_particles_ly : m_SimData.bd_particles_uy;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          const Vec3<float> pressure = (ppressure / (pden * pden)) * m_SpikyKernel.gradW(r);
                                          pressure_accel += pressure;
                                      }
                                  }


                                  // => lz/uz
                                  if(ppos[2] < valid_lz || ppos[2] > valid_uz)
                                  {
                                      const Vec3<float> ppos_scaled = ppos - m_SimParams.kernelRadius * Vec3<float>(floor(ppos[0] / m_SimParams.kernelRadius),
                                                                                                                    floor(ppos[1] / m_SimParams.kernelRadius),
                                                                                                                    0);


                                      const Vec_Vec3& bparticles = (ppos[2] < valid_lz) ? m_SimData.bd_particles_lz : m_SimData.bd_particles_uz;

                                      for(const Vec3& qpos : bparticles)
                                      {
                                          const Vec3<float> r = qpos - ppos_scaled;
                                          const Vec3<float> pressure = (ppressure / (pden * pden)) * m_SpikyKernel.gradW(r);
                                          pressure_accel += pressure;
                                      }
                                  }
                              }
                              // <= end boundary density correction
                              ////////////////////////////////////////////////////////////////////////////////
                              m_SimData.pressureAcc[p] = pressure_accel * m_SimParams.pressureStiffness;
                              /*if(p < 1000)
                                  printf("p = %d, preacc = %f,  %f, %f\n", p, pressure_accel[0], pressure_accel[1], pressure_accel[2]);*/
                          }
                      }); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::updateVelocity(float timeStep)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, velocity.size()), [&, timeStep](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              m_SimData.velocity[p] += m_SimData.pressureAcc[p] * timeStep;
                          }
                      }); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void SPHSolver::computeViscosity()
{
    static Vec_Vec3<float> diffusedVelocity;
    diffusedVelocity.resize(m_SimData.velocity.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_SimData.particles.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              const Vec3& ppos = m_SimData.particles[p];
                              const Vec3& pvel = m_SimData.velocity[p];
                              //const float pden = density[p];
                              const Vec3i pcellId = m_Grid3D.getCellIdx<int>(ppos);

                              Vec3<float> diffuse_vel = Vec3<float>(0);

                              for(int lk = -1; lk <= 1; ++lk)
                              {
                                  for(int lj = -1; lj <= 1; ++lj)
                                  {
                                      for(int li = -1; li <= 1; ++li)
                                      {
                                          const Vec3i cellId = pcellId + Vec3i(li, lj, lk);

                                          if(!m_Grid3D.isValidCell<int>(cellId))
                                          {
                                              continue;
                                          }

                                          for(UInt32 q : m_SimData.cellParticles(cellId))
                                          {
                                              if((UInt32)p == q)
                                              {
                                                  continue;
                                              }

                                              const Vec3& qpos = m_SimData.particles[q];
                                              const Vec3& qvel = m_SimData.velocity[q];
                                              const float qden = m_SimData.density[q];
                                              const Vec3<float> r = qpos - ppos;

                                              diffuse_vel += (1.0 / qden) * (qvel - pvel) * m_CubicKernel.W(r);
                                          }
                                      }
                                  }
                              }
                              diffusedVelocity[p] = diffuse_vel;
                          }
                      }); // end parallel_for


    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_SimData.velocity.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              m_SimData.velocity[p] += diffusedVelocity[p] * m_SimParams.viscosity;
                          }
                      }); // end parallel_for
}
