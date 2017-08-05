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

#undef min
#undef max

#include "FLIPSolver.h"

#include <Banana/ParallelHelpers/ParallelObjects.h>

#include <tbb/tbb.h>

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::make_ready()
{
    Timer timer;
    timer.tick();

    ////////////////////////////////////////////////////////////////////////////////
    u.resize(num_cells_x + 1, num_cells_y, num_cells_z);
    u_old.resize(num_cells_x + 1, num_cells_y, num_cells_z);
    du.resize(num_cells_x + 1, num_cells_y, num_cells_z);
    temp_u.resize(num_cells_x + 1, num_cells_y, num_cells_z);
    u_weights.resize(num_cells_x + 1, num_cells_y, num_cells_z);
    u_valid.resize(num_cells_x + 1, num_cells_y, num_cells_z);
    old_valid_u.resize(num_cells_x + 1, num_cells_y, num_cells_z);

    v.resize(num_cells_x, num_cells_y + 1, num_cells_z);
    v_old.resize(num_cells_x, num_cells_y + 1, num_cells_z);
    dv.resize(num_cells_x, num_cells_y + 1, num_cells_z);
    temp_v.resize(num_cells_x, num_cells_y + 1, num_cells_z);
    v_weights.resize(num_cells_x, num_cells_y + 1, num_cells_z);
    v_valid.resize(num_cells_x, num_cells_y + 1, num_cells_z);
    old_valid_v.resize(num_cells_x, num_cells_y + 1, num_cells_z);

    w.resize(num_cells_x, num_cells_y, num_cells_z + 1);
    w_old.resize(num_cells_x, num_cells_y, num_cells_z + 1);
    dw.resize(num_cells_x, num_cells_y, num_cells_z + 1);
    temp_w.resize(num_cells_x, num_cells_y, num_cells_z + 1);
    w_weights.resize(num_cells_x, num_cells_y, num_cells_z + 1);
    w_valid.resize(num_cells_x, num_cells_y, num_cells_z + 1);
    old_valid_w.resize(num_cells_x, num_cells_y, num_cells_z + 1);

    fluid_sdf.resize(num_cells_x, num_cells_y, num_cells_z);
    sdf_radius = cell_size * 1.01 * (float)(sqrt(3.0) / 2.0);

    if(max_num_particles != particles.size())
    {
        // reserve only, not resize
        particles.reserve(max_num_particles);
        velocity.reserve(max_num_particles);
    }

    ////////////////////////////////////////////////////////////////////////////////
    timer.tock();
    monitor.print_log("Allocate solver memory: " + timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    solver_initalized = true;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::init_step()
{
    collect_particles_to_cell();
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
bool FLIPSolver::advance_frame(float timestep, int frame)
{
    if(frame < solver_start_frame)
    {
        monitor.print_log("Skip simulation for frame " +
                          NumberUtils::format_with_commas(frame) +
                          "(Start at frame: " + NumberUtils::format_with_commas(solver_start_frame) + ")");
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    float        t            = 0;
    int          substepCount = 0;
    static Timer step_timer;


    while(t < timestep)
    {
        step_timer.tick();

        ////////////////////////////////////////////////////////////////////////////////
        float substep = fmin(get_CFL_timestep(), timestep - t);

        init_step();
        advance_velocity(substep);
        move_particles(substep);

        last_timestep = substep;

        ////////////////////////////////////////////////////////////////////////////////
        step_timer.tock();
        ++substepCount;

        monitor.print_log("Finished step " + NumberUtils::format_with_commas(substepCount) +
                          " of size " + NumberUtils::format_to_scientific(substep) +
                          "(" + NumberUtils::format_with_commas(substep / timestep * 100) + "% of the frame, to " +
                          NumberUtils::format_with_commas(100 * (t + substep) / timestep) +
                          "% of the frame).");
        monitor.print_log("Substep time: " + step_timer.get_run_time());
        monitor.new_line();

        t += substep;
    } // end while

    ////////////////////////////////////////////////////////////////////////////////
    return true;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::advance_velocity(float timestep)
{
    if(particles.size() == 0)
    {
        return;
    }

    __NOODLE_ASSERT(solver_initalized);

    static Timer timer;

    ////////////////////////////////////////////////////////////////////////////////
    //Compute finite-volume type face area weight for each velocity sample.
    static bool weight_computed = false;

    if(!weight_computed)
    {
        timer.tick();
        tbb::parallel_invoke([&]
        {
            compute_weights_u();
        },
                             [&]
        {
            compute_weights_v();
        },
                             [&]
        {
            compute_weights_w();
        });
        timer.tock();
        monitor.print_log("Compute cell weights: " + timer.get_run_time());
        weight_computed = true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    if(add_repulsive_velocity) // repulsive force
    {
        timer.tick();
        compute_repulsive_velocity(timestep);
        timer.tock();
        monitor.print_log("Add repulsive force to particles: " +
                          timer.get_run_time());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Transfer velocity from particle to grid
    timer.tick();
    //    tbb::parallel_invoke([&] {  velocity_to_grid_u(); },
    //                         [&] {  velocity_to_grid_v(); },
    //                         [&] {  velocity_to_grid_w(); });
    //    particle_velocity_to_grid();
    velocity_to_grid();
    timer.tock();
    monitor.print_log("Interpolate velocity from particles to grid: " +
                      timer.get_run_time());


    ////////////////////////////////////////////////////////////////////////////////
    timer.tick();
    tbb::parallel_invoke([&]
    {
        extrapolate_velocity(u, temp_u, u_valid, old_valid_u);
    },
                         [&]
    {
        extrapolate_velocity(v, temp_v, v_valid, old_valid_v);
    },
                         [&]
    {
        extrapolate_velocity(w, temp_w, w_valid, old_valid_w);
    });
    timer.tock();
    monitor.print_log("Extrapolate interpolated velocity: " +
                      timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    timer.tick();
    tbb::parallel_invoke([&]
    {
        constrain_velocity_u();
    },
                         [&]
    {
        constrain_velocity_v();
    },
                         [&]
    {
        constrain_velocity_w();
    });
    timer.tock();
    monitor.print_log("Constrain interpolated velocity: " +
                      timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    // backup grid velocity
    timer.tick();
    backup_grid_velocity();
    timer.tock();
    monitor.print_log("Backup grid velocity: " + timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    // add gravity
    if(has_gravity)
    {
        timer.tick();
        add_gravity(timestep);
        timer.tock();
        monitor.print_log("Add gravity: " + timer.get_run_time());
    }

    ////////////////////////////////////////////////////////////////////////////////
    monitor.print_log("Pressure projection...");
    timer.tick();
    pressure_projection(timestep);
    timer.tock();
    monitor.print_log("Pressure projection total time: " +
                      timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    //Pressure projection only produces valid velocities in faces with non-zero associated face area.
    //Because the advection step may interpolate from these invalid faces,
    //we must extrapolate velocities from the fluid domain into these invalid faces.
    timer.tick();
    tbb::parallel_invoke([&]
    {
        extrapolate_velocity(u, temp_u, u_valid, old_valid_u);
    },
                         [&]
    {
        extrapolate_velocity(v, temp_v, v_valid, old_valid_v);
    },
                         [&]
    {
        extrapolate_velocity(w, temp_w, w_valid, old_valid_w);
    });
    timer.tock();
    monitor.print_log("Grid velocity extrapolation: " + timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    //For extrapolated velocities, replace the normal component with
    //that of the object.
    timer.tick();
    tbb::parallel_invoke([&]
    {
        constrain_velocity_u();
    },
                         [&]
    {
        constrain_velocity_v();
    },
                         [&]
    {
        constrain_velocity_w();
    });

    timer.tock();
    monitor.print_log("Constrain boundary grid velocities: " +
                      timer.get_run_time());

    ////////////////////////////////////////////////////////////////////////////////
    timer.tick();
    compute_changed_velocity();
    update_particle_velocity();
    timer.tock();
    monitor.print_log("Interpolate velocity from to grid to particles: " +
                      timer.get_run_time());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::move_particles(float dt)
{
    static Timer                     timer;
    static tbb::affinity_partitioner ap;

    timer.tick();
    tbb::parallel_for(tbb::blocked_range<size_t>(0, particles.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              Vec3 ppos = particles[p] + velocity[p] * dt;

                              //check boundaries and project exterior particles back in
                              float phi_val = interpolate_value_linear((ppos - domain_bmin) / cell_size, sdf_boundary);

                              if(phi_val < 0)
                              {
                                  Vec3 grad;
                                  interpolate_gradient(grad, (ppos - domain_bmin) / cell_size, sdf_boundary);

                                  if(glm::length2(grad) > 0)
                                  {
                                      grad = glm::normalize(grad);
                                  }

                                  ppos -= phi_val * grad;
                              }

                              particles[p] = ppos;
                          }
                      }, ap); // end parallel_for

    timer.tock();
    monitor.print_log("Move particles: " + timer.get_run_time());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::save_frame(int frame)
{
    for(auto item : dataIOs)
    {
        DataFile data_file = item.first;
        DataIO*  dataIO    = dynamic_cast<DataIO*>(item.second);
        __NOODLE_ASSERT(dataIO != nullptr);


        switch(data_file)
        {
            case DataFile::FramePosition:
            {
                dataIO->reset_buffer();
                dataIO->getBuffer().push_back(static_cast<UInt32>(particles.size()));
                dataIO->getBuffer().push_back_to_float(particle_radius);
                dataIO->getBuffer().push_back_to_float_array(particles, false);
                dataIO->flush_buffer_async(frame);
            }
            break;

            case DataFile::FrameVelocity:
            {
                dataIO->reset_buffer();
                dataIO->getBuffer().push_back_to_float_array(velocity);
                dataIO->flush_buffer_async(frame);
            }
            break;

            default:
                ; // nothing
        } // end switch
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::save_state(int frame)
{
    for(auto item : dataIOs)
    {
        DataFile data_file = item.first;
        DataIO*  dataIO    = dynamic_cast<DataIO*>(item.second);
        __NOODLE_ASSERT(dataIO != nullptr);


        switch(data_file)
        {
            case DataFile::StatePosition:
            {
                dataIO->reset_buffer();
                dataIO->getBuffer().push_back(static_cast<UInt32>(particles.size()));
                dataIO->getBuffer().push_back(particle_radius);
                dataIO->getBuffer().push_back(particles, false);
                dataIO->flush_buffer_async(frame);
            }
            break;

            case DataFile::StateVelocity:
            {
                dataIO->reset_buffer();
                dataIO->getBuffer().push_back(velocity);
                dataIO->flush_buffer_async(frame);
            }
            break;

            default:
                ; // nothing
        } // end switch
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
int FLIPSolver::load_latest_state()
{
    __NOODLE_CHECK_MAP_ITEM(dataIOs, DataFile::StatePosition);
    __NOODLE_CHECK_MAP_ITEM(dataIOs, DataFile::StateVelocity);

    ////////////////////////////////////////////////////////////////////////////////
    int latest_frame = get_latest_state_index();

    ////////////////////////////////////////////////////////////////////////////////
    // position
    float particle_radius;
    dataIOs[DataFile::StatePosition]->load_file_index(latest_frame);
    dataIOs[DataFile::StatePosition]->getBuffer().get_data<float>(particle_radius, sizeof(UInt32));
    __NOODLE_ASSERT_APPROX_NUMBERS(particleParams->particle_radius, particle_radius, 1e-8);

    dataIOs[DataFile::StatePosition]->getBuffer().get_data<UInt32>(particleParams->num_particles, 0);
    particleParams->num_active_particles = particleParams->num_particles;

    if(particleParams->max_num_particles == 0)
    {
        particleParams->max_num_particles = particleParams->num_particles;
    }

    dataIOs[DataFile::StatePosition]->getBuffer().get_data<float>(particles, sizeof(UInt32) + sizeof(float), particleParams->num_particles);

    // velocity
    dataIOs[DataFile::StateVelocity]->load_file_index(latest_frame);
    dataIOs[DataFile::StateVelocity]->getBuffer().get_data<float>(velocity);
    assert(velocity.size() == particles.size());

    ////////////////////////////////////////////////////////////////////////////////
    return latest_frame;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
int FLIPSolver::get_latest_state_index()
{
    int latest_state_pos = dataIOs[DataFile::StatePosition]->find_latest_file_index(params->sim_params()->final_frame);
    int latest_state_vel = dataIOs[DataFile::StateVelocity]->find_latest_file_index(params->sim_params()->final_frame);

    if(latest_state_pos < 0 || latest_state_pos != latest_state_vel)
    {
        return -1;
    }

    return latest_state_pos;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::setup_data_io()
{
    dataIOs[DataFile::StatePosition] = new DataIO(params->dataIO_params()->data_path, "FLIPState", "state", "pos");
    dataIOs[DataFile::StateVelocity] = new DataIO(params->dataIO_params()->data_path, "FLIPState", "state", "vel");

    dataIOs[DataFile::FramePosition] = new DataIO(params->dataIO_params()->data_path, "FLIPFrame", "frame", "pos");
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::compute_cfl_timestep()
{
    if(particles.size() == 0)
    {
        cfl_timestep = 1.0;
    }

    static tbb::affinity_partitioner apu;
    static tbb::affinity_partitioner apv;
    static tbb::affinity_partitioner apw;
    array3_max_abs<float>            mu(u);
    array3_max_abs<float>            mv(v);
    array3_max_abs<float>            mw(w);

    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, u.a.size()), mu, apu);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, v.a.size()), mv, apv);
    tbb::parallel_reduce(tbb::blocked_range<size_t>(0, w.a.size()), mw, apw);

    float maxvel = MathUtils::max(mu.result, mv.result, mw.result);

    cfl_timestep = fabs(maxvel) > SMALL_NUMBER ? (cell_size / maxvel) :
                   default_timestep;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::add_to_grid_node(tbb::mutex& mutex, Array3_float& grid, const Vec3i cellId, float val)
{
    mutex.lock();

    grid(cellId) += val;

    mutex.unlock();
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::particle_velocity_to_grid()
{
    const static int span     = kernel_width();
    const Vec3       span_vec = (float)span * cell_size * Vec3(1, 1, 1);

    static Array3_float sum_weight_u;
    static Array3_float sum_weight_v;
    static Array3_float sum_weight_w;

    u.set_zero();
    v.set_zero();
    w.set_zero();

    u_valid.set_zero();
    v_valid.set_zero();
    w_valid.set_zero();

    sum_weight_u.assign(u.ni, u.nj, u.nk, 0.0);
    sum_weight_v.assign(v.ni, v.nj, v.nk, 0.0);
    sum_weight_w.assign(w.ni, w.nj, w.nk, 0.0);

    static tbb::affinity_partitioner pap;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, particles.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(size_t p = r.begin(); p != r.end(); ++p)
                          {
                              const Vec3 ppos = particles[p];
                              const Vec3i pcellId = params->domain_params()->get_cell_id(ppos);

                              // loop over neighbor cells (span^3 cells)
                              for(Int32 lk = -span; lk <= span; ++lk)
                              {
                                  for(Int32 lj = -span; lj <= span; ++lj)
                                  {
                                      for(Int32 li = -span; li <= span; ++li)
                                      {
                                          const Vec3i cellId = pcellId + Vec3i(li, lj, lk);

                                          if(!params->domain_params()->is_valid_cell(cellId))
                                          {
                                              continue;
                                          }

                                          bool valid_index_u = u.is_valid_index(cellId);
                                          bool valid_index_v = v.is_valid_index(cellId);
                                          bool valid_index_w = w.is_valid_index(cellId);

                                          const Vec3 pu = Vec3(cellId[0], cellId[1] + 0.5, cellId[2] + 0.5) * cell_size +
                                                          domain_bmin;
                                          const Vec3 pv = Vec3(cellId[0] + 0.5, cellId[1], cellId[2] + 0.5) * cell_size +
                                                          domain_bmin;
                                          const Vec3 pw = Vec3(cellId[0] + 0.5, cellId[1] + 0.5, cellId[2]) * cell_size +
                                                          domain_bmin;

                                          const Vec3 puMin = pu - span_vec;
                                          const Vec3 pvMin = pv - span_vec;
                                          const Vec3 pwMin = pw - span_vec;

                                          const Vec3 puMax = pu + span_vec;
                                          const Vec3 pvMax = pv + span_vec;
                                          const Vec3 pwMax = pw + span_vec;



                                          const Vec3& ppos = particles[p];
                                          const Vec3& pvel = velocity[p];

                                          if(valid_index_u && is_inside(ppos, puMin, puMax))
                                          {
                                              const float weight = weight_kernel((ppos - pu) / cell_size);

                                              if(weight > SMALL_NUMBER)
                                              {
                                                  add_to_grid_node(u_mutex,        u,            cellId, weight * pvel[0]);
                                                  add_to_grid_node(weight_u_mutex, sum_weight_u, cellId, weight);
                                              }
                                          }

                                          if(valid_index_v && is_inside(ppos, pvMin, pvMax))
                                          {
                                              const float weight = weight_kernel((ppos - pv) / cell_size);

                                              if(weight > SMALL_NUMBER)
                                              {
                                                  add_to_grid_node(v_mutex,        v,            cellId, weight * pvel[0]);
                                                  add_to_grid_node(weight_v_mutex, sum_weight_v, cellId, weight);
                                              }
                                          }

                                          if(valid_index_w && is_inside(ppos, pwMin, pwMax))
                                          {
                                              const float weight = weight_kernel((ppos - pw) / cell_size);

                                              if(weight > SMALL_NUMBER)
                                              {
                                                  add_to_grid_node(w_mutex,        w,            cellId, weight * pvel[0]);
                                                  add_to_grid_node(weight_w_mutex, sum_weight_w, cellId, weight);
                                              }
                                          }
                                      }
                                  }
                              } // end loop over neighbor cells
                          }
                      }, pap); // end parallel_for

    static tbb::affinity_partitioner cap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z + 1),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y + 1; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x + 1; ++i)
                                  {
                                      if(u.is_valid_index(i, j, k))
                                      {
                                          const float tmp = sum_weight_u(i, j, k);

                                          if(tmp > SMALL_NUMBER)
                                          {
                                              u(i, j, k) /= tmp;
                                              u_valid(i, j, k) = 1;
                                          }
                                      }

                                      if(v.is_valid_index(i, j, k))
                                      {
                                          const float tmp = sum_weight_v(i, j, k);

                                          if(tmp > SMALL_NUMBER)
                                          {
                                              v(i, j, k) /= tmp;
                                              v_valid(i, j, k) = 1;
                                          }
                                      }

                                      if(w.is_valid_index(i, j, k))
                                      {
                                          const float tmp = sum_weight_w(i, j, k);

                                          if(tmp > SMALL_NUMBER)
                                          {
                                              w(i, j, k) /= tmp;
                                              w_valid(i, j, k) = 1;
                                          }
                                      }
                                  }
                              }
                          }
                      }, cap);
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::velocity_to_grid()
{
    const static int span     = kernel_width();
    const Vec3       span_vec = (float)span * cell_size * Vec3(1, 1, 1);

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z + 1),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y + 1; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x + 1; ++i)
                                  {
                                      const Vec3 pu = Vec3(i, j + 0.5, k + 0.5) * cell_size +
                                                      domain_bmin;
                                      const Vec3 pv = Vec3(i + 0.5, j, k + 0.5) * cell_size +
                                                      domain_bmin;
                                      const Vec3 pw = Vec3(i + 0.5, j + 0.5, k) * cell_size +
                                                      domain_bmin;

                                      const Vec3 puMin = pu - span_vec;
                                      const Vec3 pvMin = pv - span_vec;
                                      const Vec3 pwMin = pw - span_vec;

                                      const Vec3 puMax = pu + span_vec;
                                      const Vec3 pvMax = pv + span_vec;
                                      const Vec3 pwMax = pw + span_vec;

                                      float sum_weight_u = 0.0;
                                      float sum_weight_v = 0.0;
                                      float sum_weight_w = 0.0;

                                      float sum_u = 0.0;
                                      float sum_v = 0.0;
                                      float sum_w = 0.0;

                                      bool valid_index_u = u.is_valid_index(i, j, k);
                                      bool valid_index_v = v.is_valid_index(i, j, k);
                                      bool valid_index_w = w.is_valid_index(i, j, k);

                                      // loop over neighbor cells (span^3 cells)
                                      for(Int32 lk = -span; lk <= span; ++lk)
                                      {
                                          for(Int32 lj = -span; lj <= span; ++lj)
                                          {
                                              for(Int32 li = -span; li <= span; ++li)
                                              {
                                                  const Vec3i cellId((Int32)i + li, (Int32)j + lj, (Int32)k + lk);

                                                  if(!params->domain_params()->is_valid_cell(cellId))
                                                  {
                                                      continue;
                                                  }

                                                  for(const UInt32 p : cellParticles(cellId))
                                                  {
                                                      const Vec3& ppos = particles[p];
                                                      const Vec3& pvel = velocity[p];

                                                      if(valid_index_u && is_inside(ppos, puMin, puMax))
                                                      {
                                                          const float weight = weight_kernel((ppos - pu) / cell_size);

                                                          if(weight > SMALL_NUMBER)
                                                          {
                                                              sum_u += weight * pvel[0];
                                                              sum_weight_u += weight;
                                                          }
                                                      }

                                                      if(valid_index_v && is_inside(ppos, pvMin, pvMax))
                                                      {
                                                          const float weight = weight_kernel((ppos - pv) / cell_size);

                                                          if(weight > SMALL_NUMBER)
                                                          {
                                                              sum_v += weight * pvel[1];
                                                              sum_weight_v += weight;
                                                          }
                                                      }

                                                      if(valid_index_w && is_inside(ppos, pwMin, pwMax))
                                                      {
                                                          const float weight = weight_kernel((ppos - pw) / cell_size);

                                                          if(weight > SMALL_NUMBER)
                                                          {
                                                              sum_w += weight * pvel[2];
                                                              sum_weight_w += weight;
                                                          }
                                                      }
                                                  }
                                              }
                                          }
                                      } // end loop over neighbor cells

                                      if(valid_index_u)
                                      {
                                          u(i, j, k) = (sum_weight_u > SMALL_NUMBER) ? sum_u / sum_weight_u : 0;
                                          u_valid(i, j, k) = (sum_weight_u > SMALL_NUMBER) ? 1 : 0;
                                      }

                                      if(valid_index_v)
                                      {
                                          v(i, j, k) = (sum_weight_v > SMALL_NUMBER) ? sum_v / sum_weight_v : 0;
                                          v_valid(i, j, k) = (sum_weight_v > SMALL_NUMBER) ? 1 : 0;
                                      }

                                      if(valid_index_w)
                                      {
                                          w(i, j, k) = (sum_weight_w > SMALL_NUMBER) ? sum_w / sum_weight_w : 0;
                                          w_valid(i, j, k) = (sum_weight_w > SMALL_NUMBER) ? 1 : 0;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//Compute finite-volume style face-weights for fluid from nodal signed distances
void FLIPSolver::compute_weights_u()
{
    //Compute face area fractions (using marching squares cases).
    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x + 1; ++i)
                                  {
                                      const float tmp = 1.0 - SignDistanceField::fraction_inside(sdf_boundary(i, j, k),
                                                                                                 sdf_boundary(i, j + 1, k),
                                                                                                 sdf_boundary(i, j,     k + 1),
                                                                                                 sdf_boundary(i, j + 1, k + 1));
                                      u_weights(i, j, k) = MathUtils::clamp(tmp, 0.0, 1.0);
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//Compute finite-volume style face-weights for fluid from nodal signed distances
void FLIPSolver::compute_weights_v()
{
    //Compute face area fractions (using marching squares cases).

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y + 1; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x; ++i)
                                  {
                                      const float tmp = 1 - SignDistanceField::fraction_inside(sdf_boundary(i, j, k),
                                                                                               sdf_boundary(i,     j, k + 1),
                                                                                               sdf_boundary(i + 1, j, k),
                                                                                               sdf_boundary(i + 1, j, k + 1));
                                      v_weights(i, j, k) = MathUtils::clamp(tmp, 0.0, 1.0);
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//Compute finite-volume style face-weights for fluid from nodal signed distances
void FLIPSolver::compute_weights_w()
{
    //Compute face area fractions (using marching squares cases).

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z + 1),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x; ++i)
                                  {
                                      const float tmp = 1 - SignDistanceField::fraction_inside(sdf_boundary(i, j, k),
                                                                                               sdf_boundary(i,     j + 1, k),
                                                                                               sdf_boundary(i + 1, j,     k),
                                                                                               sdf_boundary(i + 1, j + 1, k));
                                      w_weights(i, j, k) = MathUtils::clamp(tmp, 0.0, 1.0);
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::velocity_to_grid_u()
{
    const static int span = kernel_width();

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, u.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < u.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < u.ni; ++i)
                                  {
                                      const Vec3 pu = Vec3(i, j + 0.5, k + 0.5) * cell_size +
                                                      domain_bmin;
                                      const Vec3 puMin = pu - (float)span * Vec3(1, 1, 1) * cell_size;
                                      const Vec3 puMax = pu + (float)span * Vec3(1, 1, 1) * cell_size;

                                      float sum_weight = 0.0;
                                      float sum_u = 0.0;

                                      // loop over neighbor cells (18 cells)
                                      for(Int32 lk = -span; lk <= span; ++lk)
                                      {
                                          for(Int32 lj = -span; lj <= span; ++lj)
                                          {
                                              for(Int32 li = -span; li <= span - 1; ++li)
                                              {
                                                  const Vec3i cellId((Int32)i + li, (Int32)j + lj, (Int32)k + lk);

                                                  if(!params->domain_params()->is_valid_cell(cellId))
                                                  {
                                                      continue;
                                                  }

                                                  for(const UInt32 p : cellParticles(cellId))
                                                  {
                                                      const Vec3& ppos = particles[p];

                                                      if(is_outside(ppos, puMin, puMax))
                                                      {
                                                          continue;
                                                      }

                                                      const float weight = weight_kernel((ppos - pu) / cell_size);

                                                      if(weight > SMALL_NUMBER)
                                                      {
                                                          sum_u += weight * velocity[p][0];
                                                          sum_weight += weight;
                                                      }
                                                  }
                                              }
                                          }
                                      } // end loop over neighbor cells

                                      if(sum_weight > SMALL_NUMBER)
                                      {
                                          u(i, j, k) = sum_u / sum_weight;
                                          u_valid(i, j, k) = 1;
                                      }
                                      else
                                      {
                                          u(i, j, k) = 0;
                                          u_valid(i, j, k) = 0;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::velocity_to_grid_v()
{
    const static int span = kernel_width();

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, v.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < v.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < v.ni; ++i)
                                  {
                                      const Vec3 pv = Vec3(i + 0.5, j, k + 0.5) * cell_size +
                                                      domain_bmin;
                                      const Vec3 pvMin = pv - (float)span * Vec3(1, 1, 1) * cell_size;
                                      const Vec3 pvMax = pv + (float)span * Vec3(1, 1, 1) * cell_size;

                                      float sum_weight = 0.0;
                                      float sum_v = 0.0;

                                      // loop over neighbor cells (18 cells)
                                      for(Int32 lk = -span; lk <= span; ++lk)
                                      {
                                          for(Int32 lj = -span; lj <= span - 1; ++lj)
                                          {
                                              for(Int32 li = -span; li <= span; ++li)
                                              {
                                                  const Vec3i cellId((Int32)i + li, (Int32)j + lj, (Int32)k + lk);

                                                  if(!params->domain_params()->is_valid_cell(cellId))
                                                  {
                                                      continue;
                                                  }

                                                  for(const UInt32 p : cellParticles(cellId))
                                                  {
                                                      const Vec3& ppos = particles[p];

                                                      if(is_outside(ppos, pvMin, pvMax))
                                                      {
                                                          continue;
                                                      }

                                                      const float weight = weight_kernel((ppos - pv) / cell_size);

                                                      if(weight > SMALL_NUMBER)
                                                      {
                                                          sum_v += weight * velocity[p][1];
                                                          sum_weight += weight;
                                                      }
                                                  }
                                              }
                                          }
                                      } // end loop over neighbor cells

                                      if(sum_weight > SMALL_NUMBER)
                                      {
                                          v(i, j, k) = sum_v / sum_weight;
                                          v_valid(i, j, k) = 1;
                                      }
                                      else
                                      {
                                          v(i, j, k) = 0;
                                          v_valid(i, j, k) = 0;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::velocity_to_grid_w()
{
    const static int span = kernel_width();

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, w.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < w.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < w.ni; ++i)
                                  {
                                      const Vec3 pw = Vec3(i + 0.5, j + 0.5, k) * cell_size +
                                                      domain_bmin;
                                      const Vec3 pwMin = pw - (float)span * Vec3(1, 1, 1) * cell_size;
                                      const Vec3 pwMax = pw + (float)span * Vec3(1, 1, 1) * cell_size;

                                      float sum_weight = 0.0;
                                      float sum_w = 0.0;

                                      // loop over neighbor cells (18 cells)
                                      for(Int32 lk = -span; lk <= span - 1; ++lk)
                                      {
                                          for(Int32 lj = -span; lj <= span; ++lj)
                                          {
                                              for(Int32 li = -span; li <= span; ++li)
                                              {
                                                  const Vec3i cellId((Int32)i + li, (Int32)j + lj, (Int32)k + lk);

                                                  if(!params->domain_params()->is_valid_cell(cellId))
                                                  {
                                                      continue;
                                                  }

                                                  for(const UInt32 p : cellParticles(cellId))
                                                  {
                                                      const Vec3& ppos = particles[p];

                                                      if(is_outside(ppos, pwMin, pwMax))
                                                      {
                                                          continue;
                                                      }

                                                      const float weight = weight_kernel((ppos - pw) / cell_size);

                                                      if(weight > SMALL_NUMBER)
                                                      {
                                                          sum_w += weight * velocity[p][2];
                                                          sum_weight += weight;
                                                      }
                                                  }
                                              }
                                          }
                                      } // end loop over neighbor cells

                                      if(sum_weight > SMALL_NUMBER)
                                      {
                                          w(i, j, k) = sum_w / sum_weight;
                                          w_valid(i, j, k) = 1;
                                      }
                                      else
                                      {
                                          w(i, j, k) = 0;
                                          w_valid(i, j, k) = 0;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//Apply several iterations of a very simple propagation of valid velocity data in all directions
void FLIPSolver::extrapolate_velocity(Array3_float& grid, Array3_float& temp_grid,
                                      Array3c& valid, Array3c& old_valid)
{
    for(size_t i = 0; i < grid.size(); ++i)
    {
        temp_grid.a[i] = grid.a[i];
    }

    for(Int32 layers = 0; layers < 10; ++layers)
    {
        for(size_t i = 0; i < valid.size(); ++i)
        {
            old_valid.a[i] = valid.a[i];
        }

        //        for(Int32 k = 1; k < grid.nk - 1; ++k)
        tbb::parallel_for(tbb::blocked_range<UInt32>(1, grid.nk - 1),
                          [&](tbb::blocked_range<UInt32> r)
                          {
                              for(UInt32 k = r.begin(); k != r.end(); ++k)
                              {
                                  for(UInt32 j = 1; j < grid.nj - 1; ++j)
                                  {
                                      for(UInt32 i = 1; i < grid.ni - 1; ++i)
                                      {
                                          float sum = 0;
                                          int count = 0;

                                          if(!old_valid(i, j, k))
                                          {
                                              if(old_valid(i + 1, j, k))
                                              {
                                                  sum += grid(i + 1, j, k);
                                                  ++count;
                                              }

                                              if(old_valid(i - 1, j, k))
                                              {
                                                  sum += grid(i - 1, j, k);
                                                  ++count;
                                              }

                                              if(old_valid(i, j + 1, k))
                                              {
                                                  sum += grid(i, j + 1, k);
                                                  ++count;
                                              }

                                              if(old_valid(i, j - 1, k))
                                              {
                                                  sum += grid(i, j - 1, k);
                                                  ++count;
                                              }

                                              if(old_valid(i, j, k + 1))
                                              {
                                                  sum += grid(i, j, k + 1);
                                                  ++count;
                                              }

                                              if(old_valid(i, j, k - 1))
                                              {
                                                  sum += grid(i, j, k - 1);
                                                  ++count;
                                              }

                                              //If any of neighbour cells were valid,
                                              //assign the cell their average value and tag it as valid
                                              if(count > 0)
                                              {
                                                  temp_grid(i, j, k) = sum / (float)count;
                                                  valid(i, j, k) = 1;
                                              }
                                          }
                                      }
                                  }
                              }
                          }); // end parallel_for

        for(size_t i = 0; i < grid.size(); ++i)
        {
            grid.a[i] = temp_grid.a[i];
        }
    } // end for 10 layers
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//For extrapolated points, replace the normal component
//of velocity with the object velocity (in this case zero).
void FLIPSolver::constrain_velocity_u()
{
    //(At lower grid resolutions, the normal estimate from the signed
    //distance function can be poor, so it doesn't work quite as well.
    //An exact normal would do better if we had it for the geometry.)
    temp_u = u;

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, u.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < u.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < u.ni; ++i)
                                  {
                                      if(fabs(u_weights(i, j, k)) < SMALL_NUMBER)
                                      {
                                          //apply constraint
                                          const Vec3 pos = Vec3(i, j + 0.5, k + 0.5) * cell_size;
                                          Vec3 vel = get_velocity(pos + domain_bmin);
                                          Vec3 normal(0, 0, 0);

                                          interpolate_gradient(normal, pos / cell_size, sdf_boundary);
#ifdef __Using_Eigen_Lib__
                                          normal.normalize();
                                          float perp_component = vel.dot(normal);
#else
                                          normal = glm::normalize(normal);
                                          float perp_component = glm::dot(vel, normal);
#endif
                                          vel -= perp_component * normal;
                                          temp_u(i, j, k) = vel[0];
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for

    u = temp_u;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::constrain_velocity_v()
{
    temp_v = v;

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, v.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < v.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < v.ni; ++i)
                                  {
                                      if(fabs(v_weights(i, j, k)) < SMALL_NUMBER)
                                      {
                                          const Vec3 pos = Vec3(i + 0.5, j, k + 0.5) * cell_size;
                                          Vec3 vel = get_velocity(pos + domain_bmin);
                                          Vec3 normal(0, 0, 0);

                                          interpolate_gradient(normal, pos / cell_size, sdf_boundary);
#ifdef __Using_Eigen_Lib__
                                          normal.normalize();
                                          float perp_component = vel.dot(normal);
#else
                                          normal = glm::normalize(normal);
                                          float perp_component = glm::dot(vel, normal);
#endif
                                          vel -= perp_component * normal;
                                          temp_v(i, j, k) = vel[1];
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for

    v = temp_v;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::constrain_velocity_w()
{
    temp_w = w;

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, w.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < w.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < w.ni; ++i)
                                  {
                                      if(fabs(w_weights(i, j, k)) < SMALL_NUMBER)
                                      {
                                          const Vec3 pos = Vec3(i + 0.5, j + 0.5, k) * cell_size;
                                          Vec3 vel = get_velocity(pos + domain_bmin);
                                          Vec3 normal(0, 0, 0);

                                          interpolate_gradient(normal, pos / cell_size, sdf_boundary);
#ifdef __Using_Eigen_Lib__
                                          normal.normalize();
                                          float perp_component = vel.dot(normal);
#else
                                          normal = glm::normalize(normal);
                                          float perp_component = glm::dot(vel, normal);
#endif
                                          vel -= perp_component * normal;
                                          temp_w(i, j, k) = vel[2];
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for

    w = temp_w;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::backup_grid_velocity()
{
    std::copy(u.begin(), u.end(), u_old.begin());
    std::copy(v.begin(), v.end(), v_old.begin());
    std::copy(w.begin(), w.end(), w_old.begin());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::add_gravity(float dt)
{
    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z),
                      [&, dt](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y + 1; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x; ++i)
                                  {
                                      v(i, j, k) -= 9.8 * dt;
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::pressure_projection(float dt)
{
    Timer timer;

    //Estimate the liquid signed distance
    timer.tick();
    compute_fluid_sdf();
    timer.tock();
    monitor.print_log_indent("Compute liquid SDF: " + timer.get_run_time());

    timer.tick();
    compute_matrix(dt);
    timer.tock();
    monitor.print_log_indent("Compute pressure matrix: " + timer.get_run_time());

    timer.tick();
    compute_rhs();
    timer.tock();
    monitor.print_log_indent("Compute RHS: " + timer.get_run_time());

    timer.tick();
    solve_system();
    timer.tock();
    monitor.print_log_indent("Solve pressure linear system: " + timer.get_run_time());

    timer.tick();
    tbb::parallel_invoke([&, dt]
    {
        update_velocity_u(dt);
    },
                         [&, dt]
    {
        update_velocity_v(dt);
    },
                         [&, dt]
    {
        update_velocity_w(dt);
    });
    timer.tock();
    monitor.print_log_indent("Update grid velocity: " + timer.get_run_time());
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::compute_changed_velocity()
{
    for(size_t i = 0; i < u.a.size(); ++i)
    {
        du.a[i] = u.a[i] - u_old.a[i];
    }

    for(size_t i = 0; i < v.a.size(); ++i)
    {
        dv.a[i] = v.a[i] - v_old.a[i];
    }

    for(size_t i = 0; i < w.a.size(); ++i)
    {
        dw.a[i] = w.a[i] - w_old.a[i];
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::update_particle_velocity()
{
    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, particles.size()),
                      [&](tbb::blocked_range<size_t> r)
                      {
                          for(auto p = r.begin(); p != r.end(); ++p)
                          {
                              const Vec3& ppos = particles[p];
                              const Vec3& pvel = velocity[p];

                              const Vec3& newVel = get_velocity(ppos);
                              const Vec3& dVel = get_velocity_changed(ppos);

                              velocity[p] = (float)(1.0 - PIC_FLIP_RATE) * newVel + (float)PIC_FLIP_RATE * (pvel + dVel);
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::compute_fluid_sdf()
{
    //grab from particles
    fluid_sdf.assign(3 * cell_size);

    static tbb::affinity_partitioner app;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, particles.size()),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 p = r.begin(); p != r.end(); ++p)
                          {
                              const Vec3i& cellId = params->domain_params()->get_cell_id(particles[p]);

                              for(Int32 k = std::max(0, cellId[2] - 1);
                                  k <= std::min(cellId[2] + 1, (Int32)num_cells_z - 1); ++k)
                              {
                                  for(Int32 j = std::max(0, cellId[1] - 1);
                                      j <= std::min(cellId[1] + 1, (Int32)num_cells_y - 1); ++j)
                                  {
                                      for(Int32 i = std::max(0, cellId[0] - 1);
                                          i <= std::min(cellId[0] + 1, (Int32)num_cells_x - 1); ++i)
                                      {
                                          const Vec3 sample_pos = Vec3(i + 0.5, j + 0.5,
                                                                       k + 0.5) * cell_size +
                                                                  domain_bmin;
#ifdef __Using_Eigen_Lib__
                                          const float test_val = (sample_pos - particles[p]).norm() - sdf_radius;
#else
                                          const float test_val = glm::length(sample_pos - particles[p]) - sdf_radius;
#endif

                                          if(test_val < fluid_sdf(i, j, k))
                                          {
                                              fluid_sdf(i, j, k) = test_val;
                                          }
                                      }
                                  }
                              }
                          }
                      }, app); // end parallel_for


    //extend phi slightly into solids (this is a simple, naive approach, but works reasonably well)
    static Array3_float phi_temp;
    phi_temp = fluid_sdf;

    static tbb::affinity_partitioner apc;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < num_cells_y; ++j)
                              {
                                  for(UInt32 i = 0; i < num_cells_x; ++i)
                                  {
                                      if(fluid_sdf(i, j, k) < 0.5 * cell_size)
                                      {
                                          const float solid_phi_val = 0.125 * (sdf_boundary(i, j, k) +
                                                                               sdf_boundary(i + 1, j,     k) +
                                                                               sdf_boundary(i,     j + 1, k) +
                                                                               sdf_boundary(i + 1, j + 1, k) +
                                                                               sdf_boundary(i,     j,     k + 1) +
                                                                               sdf_boundary(i + 1, j,     k + 1) +
                                                                               sdf_boundary(i,     j + 1, k + 1) +
                                                                               sdf_boundary(i + 1, j + 1, k + 1));

                                          if(solid_phi_val < 0)
                                          {
                                              phi_temp(i, j, k) = -0.5 * cell_size;
                                          }
                                      }
                                  }
                              }
                          }
                      }, apc); // end parallel_for

    fluid_sdf = phi_temp;
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::compute_matrix(float dt)
{
    if(matrix.m_Size != total_cells)
    {
        matrix.resize(total_cells);
    }

    matrix.zero();

    for(UInt32 k = 1; k < num_cells_z - 1; ++k)
    {
        for(UInt32 j = 1; j < num_cells_y - 1; ++j)
        {
            for(UInt32 i = 1; i < num_cells_x - 1; ++i)
            {
                UInt32 cellId = i + num_cells_x * j +
                                num_cells_x *
                                num_cells_y * k;

                const float center_phi = fluid_sdf(i, j, k);

                if(center_phi < 0)
                {
                    //right neighbour
                    float       term      = u_weights(i + 1, j, k) * dt;
                    const float right_phi = fluid_sdf(i + 1, j, k);

                    if(right_phi < 0)
                    {
                        matrix.add_to_element(cellId, cellId,     term);
                        matrix.add_to_element(cellId, cellId + 1, -term);
                    }
                    else
                    {
                        float theta = SignDistanceField::fraction_inside(center_phi, right_phi);

                        if(theta < 0.01)
                        {
                            theta = 0.01;
                        }

                        matrix.add_to_element(cellId, cellId, term / theta);
                    }


                    //left neighbour
                    term = u_weights(i, j, k) * dt;
                    const float left_phi = fluid_sdf(i - 1, j, k);

                    if(left_phi < 0)
                    {
                        matrix.add_to_element(cellId, cellId,     term);
                        matrix.add_to_element(cellId, cellId - 1, -term);
                    }
                    else
                    {
                        float theta = SignDistanceField::fraction_inside(center_phi, left_phi);

                        if(theta < 0.01)
                        {
                            theta = 0.01;
                        }

                        matrix.add_to_element(cellId, cellId, term / theta);
                    }


                    //top neighbour
                    term = v_weights(i, j + 1, k) * dt;
                    const float top_phi = fluid_sdf(i, j + 1, k);

                    if(top_phi < 0)
                    {
                        matrix.add_to_element(cellId, cellId, term);
                        matrix.add_to_element(cellId, cellId + num_cells_x,
                                              -term);
                    }
                    else
                    {
                        float theta = SignDistanceField::fraction_inside(center_phi, top_phi);

                        if(theta < 0.01)
                        {
                            theta = 0.01;
                        }

                        matrix.add_to_element(cellId, cellId, term / theta);
                    }


                    //bottom neighbour
                    term = v_weights(i, j, k) * dt;
                    const float bot_phi = fluid_sdf(i, j - 1, k);

                    if(bot_phi < 0)
                    {
                        matrix.add_to_element(cellId, cellId, term);
                        matrix.add_to_element(cellId, cellId - num_cells_x,
                                              -term);
                    }
                    else
                    {
                        float theta = SignDistanceField::fraction_inside(center_phi, bot_phi);

                        if(theta < 0.01)
                        {
                            theta = 0.01;
                        }

                        matrix.add_to_element(cellId, cellId, term / theta);
                    }



                    //far neighbour
                    term = w_weights(i, j, k + 1) * dt;
                    const float far_phi = fluid_sdf(i, j, k + 1);

                    if(far_phi < 0)
                    {
                        matrix.add_to_element(cellId, cellId, term);
                        matrix.add_to_element(cellId,
                                              cellId + num_cells_x *
                                              num_cells_y,
                                              -term);
                    }
                    else
                    {
                        float theta = SignDistanceField::fraction_inside(center_phi, far_phi);

                        if(theta < 0.01)
                        {
                            theta = 0.01;
                        }

                        matrix.add_to_element(cellId, cellId, term / theta);
                    }


                    //near neighbour
                    term = w_weights(i, j, k) * dt;
                    const float near_phi = fluid_sdf(i, j, k - 1);

                    if(near_phi < 0)
                    {
                        matrix.add_to_element(cellId, cellId, term);
                        matrix.add_to_element(cellId,
                                              cellId - num_cells_x *
                                              num_cells_y,
                                              -term);
                    }
                    else
                    {
                        float theta = SignDistanceField::fraction_inside(center_phi, near_phi);

                        if(theta < 0.01)
                        {
                            theta = 0.01;
                        }

                        matrix.add_to_element(cellId, cellId, term / theta);
                    }
                } // end if(centre_phi < 0)
            }
        }
    }

    //    matrix.make_symmetry();
    //    matrix.write_debug();
    //    checkMatrixSymmetry();
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::compute_rhs()
{
    if(rhs.size() != total_cells)
    {
        rhs.resize(total_cells);
    }

    rhs.assign(rhs.size(), 0);


    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, num_cells_z - 1),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 1; j < num_cells_y - 1; ++j)
                              {
                                  for(UInt32 i = 1; i < num_cells_x - 1; ++i)
                                  {
                                      UInt32 index = i + num_cells_x * j +
                                                     num_cells_x *
                                                     num_cells_y * k;

                                      float tmp = 0;

                                      const float center_phi = fluid_sdf(i, j, k);

                                      if(center_phi < 0)
                                      {
                                          tmp -= u_weights(i + 1, j, k) * u(i + 1, j, k);
                                          tmp += u_weights(i, j, k) * u(i, j, k);

                                          tmp -= v_weights(i, j + 1, k) * v(i, j + 1, k);
                                          tmp += v_weights(i, j, k) * v(i, j, k);

                                          tmp -= w_weights(i, j, k + 1) * w(i, j, k + 1);
                                          tmp += w_weights(i, j, k) * w(i, j, k);
                                      } // end if(centre_phi < 0)

                                      rhs[index] = tmp;
                                  }
                              }
                          }
                      }, ap); // end parallel_for
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::solve_system()
{
    if(pressure.size() != total_cells)
    {
        pressure.resize(total_cells);
    }

    //Solve the system using Robert Bridson's incomplete Cholesky PCG solver

    float tolerance  = 0.0;
    int   iterations = 0;

    pcgSolver.setSolverParameters(CG_RELATIVE_TOLERANCE, MAX_CG_ITERATION);
    pcgSolver.setPreconditioners(MICCL0_SYMMETRIC);
//    bool success = pcgSolver.solve_precond(matrix, rhs, pressure, tolerance, iterations);
    bool success = pcgSolver.solve_precond(matrix, rhs, pressure, tolerance, iterations);

    monitor.print_log_indent("Conjugate Gradient iterations: " +
                             NumberUtils::format_with_commas(iterations) + ", final tolerance: " +
                             NumberUtils::format_to_scientific(tolerance));

    ////////////////////////////////////////////////////////////////////////////////
    Monitor::check_critical_condition(success, "Solution solve failed!");
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::update_velocity_u(float dt)
{
    //Apply the velocity update
    u_valid.assign(0);

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, u.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < u.nj; ++j)
                              {
                                  for(UInt32 i = 1; i < u.ni - 1; ++i)
                                  {
                                      const UInt32 index = i + j * num_cells_x + k *
                                                           num_cells_x *
                                                           num_cells_y;

                                      if(u_weights(i, j, k) > 0 && (fluid_sdf(i, j, k) < 0 || fluid_sdf(i - 1, j, k) < 0))
                                      {
                                          float theta = 1;

                                          if(fluid_sdf(i, j, k) >= 0 || fluid_sdf(i - 1, j, k) >= 0)
                                          {
                                              theta = SignDistanceField::fraction_inside(fluid_sdf(i - 1, j, k), fluid_sdf(i, j, k));
                                          }

                                          if(theta < 0.01)
                                          {
                                              theta = 0.01;
                                          }

                                          u(i, j, k) -= dt * (float)(pressure[index] - pressure[index - 1]) / theta;
                                          u_valid(i, j, k) = 1;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for

    for(size_t i = 0; i < u_valid.a.size(); ++i)
    {
        if(u_valid.a[i] == 0)
        {
            u.a[i] = 0;
        }
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::update_velocity_v(float dt)
{
    v_valid.assign(0);

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(0, v.nk),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 1; j < v.nj - 1; ++j)
                              {
                                  for(UInt32 i = 0; i < v.ni; ++i)
                                  {
                                      const UInt32 index = i + j * num_cells_x + k *
                                                           num_cells_x *
                                                           num_cells_y;

                                      if(v_weights(i, j, k) > 0 && (fluid_sdf(i, j, k) < 0 || fluid_sdf(i, j - 1, k) < 0))
                                      {
                                          float theta = 1;

                                          if(fluid_sdf(i, j, k) >= 0 || fluid_sdf(i, j - 1, k) >= 0)
                                          {
                                              theta = SignDistanceField::fraction_inside(fluid_sdf(i, j - 1, k), fluid_sdf(i, j, k));
                                          }

                                          if(theta < 0.01)
                                          {
                                              theta = 0.01;
                                          }

                                          v(i, j, k) -= dt * (float)(pressure[index] - pressure[index -
                                                                                                num_cells_x]) /
                                                        theta;
                                          v_valid(i, j, k) = 1;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for


    for(size_t i = 0; i < v_valid.a.size(); ++i)
    {
        if(v_valid.a[i] == 0)
        {
            v.a[i] = 0;
        }
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
void FLIPSolver::update_velocity_w(float dt)
{
    w_valid.assign(0);

    static tbb::affinity_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<UInt32>(1, w.nk - 1),
                      [&](tbb::blocked_range<UInt32> r)
                      {
                          for(UInt32 k = r.begin(); k != r.end(); ++k)
                          {
                              for(UInt32 j = 0; j < w.nj; ++j)
                              {
                                  for(UInt32 i = 0; i < w.ni; ++i)
                                  {
                                      const UInt32 index = i + j * num_cells_x + k *
                                                           num_cells_x *
                                                           num_cells_y;

                                      if(w_weights(i, j, k) > 0 && (fluid_sdf(i, j, k) < 0 || fluid_sdf(i, j, k - 1) < 0))
                                      {
                                          float theta = 1;

                                          if(fluid_sdf(i, j, k) >= 0 || fluid_sdf(i, j, k - 1) >= 0)
                                          {
                                              theta = SignDistanceField::fraction_inside(fluid_sdf(i, j, k - 1), fluid_sdf(i, j, k));
                                          }

                                          if(theta < 0.01)
                                          {
                                              theta = 0.01;
                                          }

                                          w(i, j, k) -= dt * (float)(pressure[index]
                                                                     - pressure[index
                                                                                - num_cells_x * num_cells_y])
                                                        / theta;
                                          w_valid(i, j, k) = 1;
                                      }
                                  }
                              }
                          }
                      }, ap); // end parallel_for

    for(size_t i = 0; i < w_valid.a.size(); ++i)
    {
        if(w_valid.a[i] == 0)
        {
            w.a[i] = 0;
        }
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//Vec3 FLIPSolver::get_old_velocity(const Vec3& position)
//{
//    float vx = interpolate_value((position - domain_bmin) / cell_size -
//                                Vec3(0, 0.5, 0.5), u_old);
//    float vy = interpolate_value((position - domain_bmin) / cell_size -
//                                Vec3(0.5, 0, 0.5), v_old);
//    float vz = interpolate_value((position - domain_bmin) / cell_size -
//                                Vec3(0.5, 0.5, 0), w_old);

//    return Vec3(vx, vy, vz);
//}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
Vec3 FLIPSolver::get_velocity_changed(const Vec3& position)
{
    float changed_vu = interpolate_value((position - domain_bmin) / cell_size -
                                         Vec3(0,   0.5, 0.5), du);
    float changed_vv = interpolate_value((position - domain_bmin) / cell_size -
                                         Vec3(0.5, 0,   0.5), dv);
    float changed_vw = interpolate_value((position - domain_bmin) / cell_size -
                                         Vec3(0.5, 0.5, 0), dw);

    return Vec3(changed_vu, changed_vv, changed_vw);
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//Interpolate velocity from the MAC grid.
Vec3 FLIPSolver::get_velocity(const Vec3& position)
{
    return Vec3(get_velocity_u(position), get_velocity_v(position),
                get_velocity_w(position));
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float FLIPSolver::get_velocity_u(const Vec3& position)
{
    return interpolate_value((position - domain_bmin) / cell_size -
                             Vec3(0, 0.5, 0.5), u);
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float FLIPSolver::get_velocity_v(const Vec3& position)
{
    return interpolate_value((position - domain_bmin) / cell_size -
                             Vec3(0.5, 0, 0.5), v);
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float FLIPSolver::get_velocity_w(const Vec3& position)
{
    return interpolate_value((position - domain_bmin) / cell_size -
                             Vec3(0.5, 0.5, 0), w);
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
int FLIPSolver::kernel_width()
{
    switch(interpolation_kernel)
    {
        case FLIPInterpolationKernel::LinearKernel:
            return 1;

        case FLIPInterpolationKernel::CubicSplineKernel:
            return 2;

        default:
            __NOODLE_INVALID_SWITCH_DEFAULT_VALUE
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float FLIPSolver::weight_kernel(const Vec3& dxdydz)
{
    switch(interpolation_kernel)
    {
        case FLIPInterpolationKernel::LinearKernel:
            return MathUtils::tril_kernel(dxdydz[0], dxdydz[1], dxdydz[2]);

        case FLIPInterpolationKernel::CubicSplineKernel:
            return MathUtils::cubic_spline_kernel_3d(dxdydz[0], dxdydz[1], dxdydz[2]);

        default:
            __NOODLE_INVALID_SWITCH_DEFAULT_VALUE
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
float FLIPSolver::interpolate_value(const Vec3&         point,
                                    const Array3_float& grid)
{
    switch(interpolation_kernel)
    {
        case FLIPInterpolationKernel::LinearKernel:
            return interpolate_value_linear(point, grid);

        case FLIPInterpolationKernel::CubicSplineKernel:
            return interpolate_value_cubic_bspline(point, grid);

        default:
            __NOODLE_INVALID_SWITCH_DEFAULT_VALUE
    }
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
bool FLIPSolver::is_inside(const Vec3& pos, const Vec3& bMin,
                           const Vec3& bMax)
{
    return !is_outside(pos, bMin, bMax);
}

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
bool FLIPSolver::is_outside(const Vec3& pos, const Vec3& bMin,
                            const Vec3& bMax)
{
    return (pos[0] < bMin[0] || pos[1] < bMin[1] || pos[2] < bMin[2] ||
            pos[0] > bMax[0] || pos[1] > bMax[1] || pos[2] > bMax[2]);
}
