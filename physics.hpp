#pragma once
#include "collision_grid.hpp"
#include "thread_pool.hpp"
#include "utils.hpp"
#include <vector>


struct PhysicSolver
{
    //std::vector<PhysicObject> objects;
    std::vector<float> positions;
    std::vector<float> last_positions;
    std::vector<float> accelerations;

    int32_t numParticles = 0;
    
    CollisionGrid          grid;
    int32_t                WIDTH;
    int32_t                HEIGHT;
    float                  gravityX = 0.0f;
    float                  gravityY = 20.f;
    float radius;
    int32_t scaledWIDTH;
    int32_t scaledHEIGHT;
    float scalingFactor;

    // Simulation solving pass count
    uint32_t        sub_steps;
    tp::ThreadPool& thread_pool;

    PhysicSolver(int32_t WIDTH_, int32_t HEIGHT_, int32_t scaledWIDTH_, int32_t scaledHEIGHT_, float radius_, tp::ThreadPool& tp)
        : grid{WIDTH_, HEIGHT_}
        , WIDTH(WIDTH_), HEIGHT(HEIGHT_)
        , sub_steps{8}
        , thread_pool{tp}
        , radius(radius_)
        , scaledWIDTH(scaledWIDTH_)
        , scaledHEIGHT(scaledHEIGHT_)
        , scalingFactor(2 * radius_)
    {
        grid.clear();
    }

    // Checks if two atoms are colliding and if so create a new contact
    void solveContact(uint32_t index, uint32_t otherIndex)
    {
        constexpr float response_coef = 1.0f;
        constexpr float eps           = 0.0001f;
        const float o2_o1X  = positions[2 * index] - positions[2 * otherIndex];
        const float o2_o1Y  = positions[2 * index + 1] - positions[2 * otherIndex + 1];

        const float dist2 = o2_o1X * o2_o1X + o2_o1Y * o2_o1Y;

        const float moveDist = 2 * radius;
        const float checkSeperationDist = moveDist * moveDist;

        if (dist2 < checkSeperationDist && dist2 > eps) {
            const float dist          = sqrt(dist2);
            // Radius are all equal to 1.0f
            const float delta = response_coef * 0.5f * (moveDist - dist) / dist;
            const float col_vecX = o2_o1X * delta;
            const float col_vecY = o2_o1Y * delta;

            positions[2 * index] += col_vecX;
            positions[2 * index + 1] += col_vecY;

            positions[2 * otherIndex] -= col_vecX;
            positions[2 * otherIndex + 1] -= col_vecY;
        }
    }

    void checkAtomCellCollisions(uint32_t atom_idx, const CollisionCell& c)
    {
        for (uint32_t i{0}; i < c.objects_count; ++i) {
            solveContact(atom_idx, c.objects[i]);
        }
    }

    void processCell(const CollisionCell& c, uint32_t index)
    {
        for (uint32_t i{0}; i < c.objects_count; ++i) {
            const uint32_t atom_idx = c.objects[i];
            checkAtomCellCollisions(atom_idx, grid.data[index - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index]);
            checkAtomCellCollisions(atom_idx, grid.data[index + 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height    ]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height + 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height    ]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height + 1]);
        }
    }

    void solveCollisionThreaded(uint32_t start, uint32_t end)
    {
        for (uint32_t idx{start}; idx < end; ++idx) {
            processCell(grid.data[idx], idx);
        }
    }

    // Find colliding atoms
    void solveCollisions()
    {
        // Multi-thread grid
        const uint32_t thread_count = thread_pool.m_thread_count;
        const uint32_t slice_count  = thread_count * 2;
        const uint32_t slice_size   = (grid.width / slice_count) * grid.height;
        const uint32_t last_cell    = (2 * (thread_count - 1) + 2) * slice_size;
        // Find collisions in two passes to avoid data races

        // First collision pass
        for (uint32_t i{0}; i < thread_count; ++i) {
            thread_pool.addTask([this, i, slice_size]{
                uint32_t const start{2 * i * slice_size};
                uint32_t const end  {start + slice_size};
                solveCollisionThreaded(start, end);
            });
        }
        // Eventually process rest if the world is not divisible by the thread count
        if (last_cell < grid.data.size()) {
            thread_pool.addTask([this, last_cell]{
                solveCollisionThreaded(last_cell, to<uint32_t>(grid.data.size()));
            });
        }
        thread_pool.waitForCompletion();
        // Second collision pass
        for (uint32_t i{0}; i < thread_count; ++i) {
            thread_pool.addTask([this, i, slice_size]{
                uint32_t const start{(2 * i + 1) * slice_size};
                uint32_t const end  {start + slice_size};
                solveCollisionThreaded(start, end);
            });
        }
        thread_pool.waitForCompletion();
    }

    // Add a new object to the solver
    void createObject(float posX, float posY)
    {   
        numParticles++;
        positions.emplace_back(posX);
        positions.emplace_back(posY);
        last_positions.emplace_back(posX);
        last_positions.emplace_back(posY);
        accelerations.emplace_back(0.f);
        accelerations.emplace_back(0.f);
    }

    void update(float dt)
    {
        // Perform the sub steps
        const float sub_dt = dt / to<float>(sub_steps);
        for (uint32_t i(sub_steps); i--;) {
            addObjectsToGrid();
            solveCollisions();
            updateObjects_multi(sub_dt);
        }
    }

    void addObjectsToGrid()
    {
        grid.clear();
        // Safety border to avoid adding object outside the grid
        uint32_t i{0};
        for (int32_t index = 0; index < numParticles; ++index) {
            if (positions[2 * index] > scalingFactor && positions[2 * index] < scaledWIDTH - scalingFactor &&
                positions[2 * index + 1] > scalingFactor && positions[2 * index + 1] < scaledHEIGHT - scalingFactor) {
                grid.addAtom(to<int32_t>(positions[2 * index] / scalingFactor), to<int32_t>(positions[2 * index + 1] / scalingFactor), i);
            }
            ++i;
        }
    }

    void updateObjects_multi(float dt)
    {
        thread_pool.dispatch(numParticles, [&](uint32_t start, uint32_t end){
            for (uint32_t i{start}; i < end; ++i) {
                // Add gravity
                accelerations[2 * i] += gravityX;
                accelerations[2 * i + 1] += gravityY * scalingFactor;
                // Apply Verlet integration
                integrate(i, dt);
                // Apply map borders collisions
                const float margin = 2.f; // 1.05f
                if (positions[2 * i] > scaledWIDTH - margin * scalingFactor) {
                    positions[2 * i] = scaledWIDTH - margin * scalingFactor;
                } else if (positions[2 * i] < margin * scalingFactor) {
                    positions[2 * i] = margin * scalingFactor;
                }

                if (positions[2 * i + 1] > scaledHEIGHT - margin * scalingFactor) {
                    positions[2 * i + 1] = scaledHEIGHT - margin * scalingFactor;

                } else if (positions[2 * i + 1] < margin * scalingFactor) {
                    positions[2 * i + 1] = margin * scalingFactor;
                }
            }
        });
    }

private:

    void setPosition(int32_t index, float posX, float posY)
    {
        positions[2 * index]      = posX;
        positions[2 * index + 1]      = posY;
        last_positions[2 * index] = posX;
        last_positions[2 * index + 1] = posY;
    }

    void integrate(int32_t index, float dt)
    {

        const float last_update_moveX = positions[2 * index] - last_positions[2 * index];
        const float last_update_moveY = positions[2 * index + 1] - last_positions[2 * index + 1];

        const float VELOCITY_DAMPING = 40.0f; // arbitrary, approximating air friction

        const float new_positionX = positions[2 * index] + last_update_moveX + (accelerations[2 * index] - last_update_moveX * VELOCITY_DAMPING) * (dt * dt);

        const float new_positionY = positions[2 * index + 1] + last_update_moveY + (accelerations[2 * index + 1] - last_update_moveY * VELOCITY_DAMPING) * (dt * dt);

        last_positions[2 * index]               = positions[2 * index];
        last_positions[2 * index + 1]           = positions[2 * index + 1];

        positions[2 * index]                    = new_positionX;
        positions[2 * index + 1]                = new_positionY;

        accelerations[2 * index]     = 0.f;
        accelerations[2 * index + 1] = 0.f;
    }

    void stop(int32_t index)
    {
        last_positions[2 * index] = positions[2 * index];
        last_positions[2 * index + 1] = positions[2 * index + 1];
    }

    void slowdown(int32_t index, float ratio)
    {
        last_positions[2 * index] = last_positions[2 * index] + ratio * (positions[2 * index] - last_positions[2 * index]);
        last_positions[2 * index + 1] = last_positions[2 * index + 1] + ratio * (positions[2 * index + 1] - last_positions[2 * index + 1]);
    }

    [[nodiscard]]
    float getSpeed(int32_t index) const
    {
        return std::sqrt((positions[2 * index] - last_positions[2 * index]) * (positions[2 * index] - last_positions[2 * index]) + (positions[2 * index + 1] - last_positions[2 * index + 1]) * (positions[2 * index + 1] - last_positions[2 * index + 1]));
    }

    [[nodiscard]]
    float getVelocityX(int32_t index) const
    {
        return positions[2 * index] - last_positions[2 * index];
    }

    [[nodiscard]]
    float getVelocityY(int32_t index) const
    {
        return positions[2 * index + 1] - last_positions[2 * index + 1];
    }

    void addVelocityX(int32_t index, const float vx)
    {
        last_positions[2 * index] -= vx;
    }

    void addVelocityY(int32_t index, const float vy)
    {
        last_positions[2 * index + 1] -= vy;
    }

    void setPositionSameSpeed(int32_t index, const float new_positionX, const float new_positionY)
    {
        const float to_lastX = last_positions[2 * index] - positions[2 * index];
        const float to_lastY = last_positions[2 * index + 1] - positions[2 * index + 1];
        positions[2 * index]               = new_positionX;
        positions[2 * index + 1]           = new_positionY;

        last_positions[2 * index]      = positions[2 * index] + to_lastX;
        last_positions[2 * index + 1]      = positions[2 * index + 1] + to_lastY;
    }

    void moveX(int32_t index, const float vx)
    {
        positions[2 * index] += vx;
    }

    void moveY(int32_t index, const float vy) 
    {
        positions[2 * index + 1] += vy;
    }

};
