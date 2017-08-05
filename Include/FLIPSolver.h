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

#include <mutex>

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class FLIPSolver : public FluidSolver
{
public:
    FLIPSolver(std::shared_ptr<ParticleSystemData>& particleData, const std::shared_ptr<SimulationParameters>& simParams) :
        FluidSolver(simParams), m_SimData(particleData) {}

    virtual float advanceFrame() override;
    virtual void  makeReady() override;

    virtual unsigned int getNumParticles() override { return static_cast<unsigned int>(m_SimData.particles.size()); }
    virtual Vec_Vec3<float>& getParticles() override { return m_SimData.particles; }
    virtual Vec_Vec3<float>& getVelocity() override { return m_SimData.velocity; }

private:
    void initStep() override;
    void advanceVelocity(float timestep) override;
    void addGravity(float dt) override;
    void moveParticles(float dt) override;
    void computeCFLTimestep() override;

    // flip virtual function
    tbb::mutex u_mutex;
    tbb::mutex v_mutex;
    tbb::mutex w_mutex;
    tbb::mutex weight_u_mutex;
    tbb::mutex weight_v_mutex;
    tbb::mutex weight_w_mutex;
    void         add_to_grid_node(tbb::mutex& mutex, Array3_float& grid, const Vec3i cellId, float val);
    virtual void particle_velocity_to_grid();
    virtual void velocity_to_grid();
    virtual void velocity_to_grid_u();
    virtual void velocity_to_grid_v();
    virtual void velocity_to_grid_w();
    virtual void update_particle_velocity();

    ////////////////////////////////////////////////////////////////////////////////
    // velocity integration functions
    void compute_weights_u();
    void compute_weights_v();
    void compute_weights_w();

    void extrapolate_velocity(Array3_float& grid, Array3_float& temp_grid, Array3c& valid,
                              Array3c& old_valid);

    void constrain_velocity_u();
    void constrain_velocity_v();
    void constrain_velocity_w();

    void backup_grid_velocity();
    void pressure_projection(float dt);

    void compute_changed_velocity();

    ////////////////////////////////////////////////////////////////////////////////
    // pressure projection functions
    void compute_fluid_sdf();

    void compute_matrix(float dt);
    void compute_rhs();
    void solve_system();

    void update_velocity_u(float dt);
    void update_velocity_v(float dt);
    void update_velocity_w(float dt);

    ////////////////////////////////////////////////////////////////////////////////
    // other utility functions
    Vec3  get_old_velocity(const Vec3& position);
    Vec3  get_velocity_changed(const Vec3& position);
    Vec3  get_velocity(const Vec3& position);
    float get_velocity_u(const Vec3& position);
    float get_velocity_v(const Vec3& position);
    float get_velocity_w(const Vec3& position);

    int   kernel_width();
    float weight_kernel(const Vec3& dxdydz);
    float interpolate_value(const Vec3& point, const Array3_float& grid);

    bool is_inside(const Vec3& pos, const Vec3& bMin, const Vec3& bMax);
    bool is_outside(const Vec3& pos, const Vec3& bMin, const Vec3& bMax);

    ////////////////////////////////////////////////////////////////////////////////
    //Fluid grid data for velocity
    Array3_float u, v, w;
    Array3_float u_old, v_old, w_old;
    Array3_float du, dv, dw;
    Array3_float u_weights, v_weights, w_weights;
    Array3c      u_valid, v_valid, w_valid;

    // temp array
    Array3_float temp_u, temp_v, temp_w;
    Array3c      old_valid_u, old_valid_v, old_valid_w;

    Array3_float fluid_sdf;
    float        sdf_radius; // radius for level set fluid

    // map data from params variable
    const UInt32              max_num_particles;
    const InterpolationKernel interpolation_kernel;
    const float               repulsive_support;
    const float               K_repulsive_force;

    ////////////////////////////////////////////////////////////////////////////////
    //Solver
    PCGSolver<float>    pcgSolver;
    SparseMatrix<float> matrix;
    std::vector<float>  rhs;
    std::vector<float>  pressure;
};
