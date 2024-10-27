#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <array>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <functional>
#include <unordered_map>

namespace tp
{

struct TaskQueue
{
    std::queue<std::function<void()>> m_tasks;
    std::mutex                        m_mutex;
    std::atomic<uint32_t>             m_remaining_tasks = 0;

    template<typename TCallback>
    void addTask(TCallback&& callback)
    {
        std::lock_guard<std::mutex> lock_guard{m_mutex};
        m_tasks.push(std::forward<TCallback>(callback));
        m_remaining_tasks++;
    }

    void getTask(std::function<void()>& target_callback)
    {
        {
            std::lock_guard<std::mutex> lock_guard{m_mutex};
            if (m_tasks.empty()) {
                return;
            }
            target_callback = std::move(m_tasks.front());
            m_tasks.pop();
        }
    }

    static void wait()
    {
        std::this_thread::yield();
    }

    void waitForCompletion() const
    {
        while (m_remaining_tasks > 0) {
            wait();
        }
    }

    void workDone()
    {
        m_remaining_tasks--;
    }
};

struct Worker
{
    uint32_t              m_id      = 0;
    std::thread           m_thread;
    std::function<void()> m_task    = nullptr;
    bool                  m_running = true;
    TaskQueue*            m_queue   = nullptr;

    Worker() = default;

    Worker(TaskQueue& queue, uint32_t id)
        : m_id{id}
        , m_queue{&queue}
    {
        m_thread = std::thread([this](){
            run();
        });
    }

    void run()
    {
        while (m_running) {
            m_queue->getTask(m_task);
            if (m_task == nullptr) {
                TaskQueue::wait();
            } else {
                m_task();
                m_queue->workDone();
                m_task = nullptr;
            }
        }
    }

    void stop()
    {
        m_running = false;
        m_thread.join();
    }
};

struct ThreadPool
{
    uint32_t            m_thread_count = 0;
    TaskQueue           m_queue;
    std::vector<Worker> m_workers;

    explicit
    ThreadPool(uint32_t thread_count)
        : m_thread_count{thread_count}
    {
        m_workers.reserve(thread_count);
        for (uint32_t i{thread_count}; i--;) {
            m_workers.emplace_back(m_queue, static_cast<uint32_t>(m_workers.size()));
        }
    }

    virtual ~ThreadPool()
    {
        for (Worker& worker : m_workers) {
            worker.stop();
        }
    }

    template<typename TCallback>
    void addTask(TCallback&& callback)
    {
        m_queue.addTask(std::forward<TCallback>(callback));
    }

    void waitForCompletion() const
    {
        m_queue.waitForCompletion();
    }

    template<typename TCallback>
    void dispatch(uint32_t element_count, TCallback&& callback)
    {
        const uint32_t batch_size = element_count / m_thread_count;
        for (uint32_t i{0}; i < m_thread_count; ++i) {
            const uint32_t start = batch_size * i;
            const uint32_t end   = start + batch_size;
            addTask([start, end, &callback](){ callback(start, end); });
        }

        if (batch_size * m_thread_count < element_count) {
            const uint32_t start = batch_size * m_thread_count;
            callback(start, element_count);
        }

        waitForCompletion();
    }
};

}

class VerletBalls {
    int numParticles;
    float radius;

    std::vector<float> positions;
    std::vector<float> oldPositions;
    std::vector<float> forces;

    float friction;
    std::vector<int> frictions;

    int tableSize;
    int numObjects;
    std::vector<int> cellCount;
    std::vector<int> particleArray;
    float spacing;
    bool interacting = false;
    int WIDTH;
    int HEIGHT;

    float restitution;
    float checkSeperationDist;
    float moveDist;
    float mouseX = 0;
    float mouseY = 0;

    std::vector<int> particleColors;

    float gravity;

    std::array<std::array<int, 3>, 100> gradient;
    std::array<std::array<int, 3>, 4> colorMap{{{0, 51, 102}, {0, 153, 204}, {102, 255, 204}, {255, 255, 255}}};
    // scientific: {0, 150, 255}, {0, 255, 0}, {255, 255, 0}, {255, 0, 0}
    // night ocean: {0, 51, 102}, {0, 153, 204}, {102, 255, 204}, {255, 255, 255}
    // sunset: {0, 0, 64}, {128, 0, 128}, {255, 128, 0}, {255, 255, 0}
    // orange to white: {102, 51, 0}, {204, 122, 0}, {255, 153, 51}, {255, 255, 255}
    // ice: {0, 102, 204}, {173, 216, 230}, {224, 255, 255}, {255, 250, 250}
    // lava: {128, 0, 0}, {255, 69, 0}, {255, 140, 0}, {255, 215, 0}
    // deep space: {0, 0, 32}, {64, 0, 128}, {128, 0, 255}, {192, 192, 255}
    // dark blue: {0, 0, 128}, {0, 128, 255}, {255, 128, 0}, {255, 255, 0}
    // lightning mcqueen: {255, 0, 0}, {255, 69, 0}, {255, 165, 0}, {255, 255, 0}
    // rainbow: {255, 0, 0}, {255, 255, 0}, {0, 255, 0}, {0, 200, 255} 

    int numRowsPerThread;
    int numMissedRows;
    int numThreads = std::thread::hardware_concurrency() < 2 ? std::thread::hardware_concurrency() : 2;
    tp::ThreadPool thread_pool;

    sf::CircleShape particleDrawer;

    sf::Font font;

    sf::Text text;

    float forceObjectRadius = 250; // 200
    std::unordered_map<int, float> forceObjectQueries;
    sf::CircleShape forceObjectDrawer;
    float checkForceObjectSeperationDist;

    float dt;

    std::vector<bool> affectedByForceObj;

    float nY;
    float nX;

public:
    VerletBalls(float WIDTH, float HEIGHT, int numParticles, float gravity, float restitution, float friction, float radius)
        : numParticles(numParticles), WIDTH(WIDTH), HEIGHT(HEIGHT), gravity(gravity), restitution(restitution), friction(friction), radius(radius), thread_pool(std::thread::hardware_concurrency() < 2 ? std::thread::hardware_concurrency() : 2) {
            font.loadFromFile("C:\\Users\\dklos\\vogue\\Vogue.ttf");
            
            text.setFont(font);
            text.setPosition(10, 10);
            text.setFillColor(sf::Color::White);
            text.setScale(0.7, 0.7);

            this->positions.resize(2 * numParticles);
            this->oldPositions.resize(2 * numParticles);
            this->forces.resize(2 * numParticles);
            this->affectedByForceObj.resize(numParticles);
            this->frictions.resize(2 * numParticles);

            this->particleDrawer.setOutlineThickness(0.f);
            this->particleDrawer.setRadius(this->radius);
            this->particleDrawer.setOrigin(this->radius, this->radius);
            this->particleDrawer.setFillColor(sf::Color(0, 150, 255));
            this->particleColors.resize(3 * numParticles);

            this->forceObjectDrawer.setOrigin(forceObjectRadius, forceObjectRadius);
            this->forceObjectDrawer.setRadius(forceObjectRadius);
            this->forceObjectDrawer.setOutlineThickness(1.f);
            this->forceObjectDrawer.setFillColor(sf::Color::Transparent);
            this->forceObjectDrawer.setOutlineColor(sf::Color::Red); 
            this->checkForceObjectSeperationDist = (this->radius + forceObjectRadius) * (this->radius + forceObjectRadius);

            this->spacing = 2 * this->radius;
            this->nY = std::ceil(1.f * HEIGHT / this->spacing);
            this->nX = std::ceil(1.f * WIDTH / this->spacing);

            this->tableSize = this->nX * this->nY;
            this->cellCount.resize(this->tableSize + 1);
            this->particleArray.resize(numParticles);

            this->numRowsPerThread = this->nX / this->numThreads / 2;
            this->numMissedRows = this->nX - (2 * this->numRowsPerThread * this->numThreads);

            // initialize particle positions
            int rowNum = std::floor(std::sqrt(numParticles));

            int seperation = 3;
            int starting_px = (WIDTH - (radius * seperation * rowNum)) / 2 + radius;
            int starting_py = (HEIGHT - (radius * seperation * rowNum)) / 2 + radius;

            int px = starting_px;
            int py = starting_py;

            int addTo = numParticles - rowNum * rowNum;

            bool offset = true;
            for (int i = 0; i < rowNum * rowNum + addTo; ++i) {
                this->positions[i * 2] = px;
                this->positions[i * 2 + 1] = py;
                this->oldPositions[2 * i] = px;
                this->oldPositions[2 * i + 1] = py;

                this->particleColors[3 * i] = 0;
                this->particleColors[3 * i + 1] = 150;
                this->particleColors[3 * i + 2] = 255;

                px += this->radius * seperation;

                if ((i + 1) % rowNum == 0) {
                    px = starting_px;
                    if (offset) {
                        px += this->radius;
                    }
                    py += this->radius * seperation;
                    offset = !offset;
                }
            }

            this->moveDist = 2 * radius;
            this->checkSeperationDist = moveDist * moveDist;

            float num_colors = colorMap.size() - 1; // number of colors - 1
            float num_steps = 1.f * gradient.size() / num_colors; //num_steps = 50 * key_range
            int index = 0;
            for (int i = 0; i < num_colors; ++i) {  // Iterate over adjacent color pairs
                for (int x = 0; x < num_steps; ++x) {
                    float t = 1.f * x / num_steps;  // Interpolation factor
                    // Linear interpolation for r, g, b values between colorMap[i] andcolorMap [i+1]
                    int r = (int)(colorMap[i][0] * (1 - t) + colorMap[i + 1][0] * t);
                    int g = (int)(colorMap[i][1] * (1 - t) + colorMap[i + 1][1] * t);
                    int b = (int)(colorMap[i][2] * (1 - t) + colorMap[i + 1][2] * t);
                    gradient[index] = std::array<int, 3>{r, g, b};
                    index++;
                }
            }
    }

private:

    int hashCoords(int xi, int yi) {
        int hash = (intCoord(xi) * 92837111) ^ (intCoord(yi) * 689287499);
        return std::abs(hash) % this->tableSize;
    }

    int intCoord(int coord) {
        // add 1 so that you dont get 0 for intCoord(xi) or intCoord(yi) in the hashCoords
        return std::floor(coord / this->spacing) + 1;
    }

    int getCell(float x, float y) {
        int xi = std::floor(x / this->spacing);
        int yi = std::floor(y / this->spacing);
        return xi * this->nY + yi;
    }

    float clamp(float x, float min, float max) {
        if (x < min) {
            return min;
        }
        else if (x > max) {
            return max;
        }
        return x;
    }

public:

    void InitializeHM() {
        std::fill(this->cellCount.begin(), this->cellCount.end(), 0);
        std::fill(this->particleArray.begin(), this->particleArray.end(), 0);
        // initialize cells in cellCount
        for (int i = 0; i < this->numParticles; ++i) {
            if (positions[2 * i] > 0 && positions[2 * i] < WIDTH && positions[2 * i + 1] > 0 && positions[2 * i + 1] < HEIGHT) {
                int cell = this->getCell(positions[2 * i], positions[2 * i + 1]);
                this->cellCount[cell]++;
            }
        }
        
        // calc partial sum
        int sum = 0;
        for (int i = 0; i < tableSize + 1; ++i) {
            sum += this->cellCount[i];
            this->cellCount[i] = sum;
        }
        this->cellCount[tableSize] = sum;
       
        // fill particle array
        for (int i = 0; i < this->numParticles; ++i) {
            if (positions[2 * i] > 0 && positions[2 * i] < WIDTH && positions[2 * i + 1] > 0 && positions[2 * i + 1] < HEIGHT) {
                int cell = this->getCell(positions[2 * i], positions[2 * i + 1]);
                cellCount[cell]--;
                particleArray[cellCount[cell]] = i;
            }
        }
    }

    void makeParticleQueries2(int startColumn, int endColumn) {
        // loop horizontally from the start row to the end row
        for (int r = startColumn; r < endColumn; ++r) {
            // loop vertically through every vertical column in each row
            for (int c = 1; c < nY - 1; ++c) {

                int cell = this->hashCoords(r * this->spacing, c * this->spacing);
                int start = this->cellCount[cell];
                int end = this->cellCount[cell + 1];

                // loop through every particle in every cell we come across
                for (int particleArrayID = start; particleArrayID < end; ++particleArrayID) {
                    int particleIndex = particleArray[particleArrayID];

                    /*this->particleColors[3 * particleIndex] = 0;
                    particleColors[3 * particleIndex + 1] = 150;
                    particleColors[3 * particleIndex + 2] = 255;*/

                    // loop over every adjacent cell, as well as the same cell to solve all collisions for that particle
                    for (int i = -1; i < 2; ++i) {
                        for (int j = -1; j < 2; ++j) {
                            int otherCell = this->getCell(this->positions[particleIndex * 2] + i * this->spacing, this->positions[particleIndex * 2 + 1] + j * this->spacing);
                            int startOther = this->cellCount[otherCell];
                            int endOther = this->cellCount[otherCell + 1];

                            // loop over all the neighbor particles in adjacent cells
                            for (int p = startOther; p < endOther; ++p) {
                                int otherParticleID = this->particleArray[p];
                    
                                if (otherParticleID == particleIndex) continue;

                                float dx = this->positions[otherParticleID * 2] - this->positions[particleIndex * 2];
                                float dy = this->positions[otherParticleID * 2 + 1] - this->positions[particleIndex * 2 + 1];
                                float d2 = dx * dx + dy * dy;
                                if (d2 > checkSeperationDist || d2 == 0.0) continue;
                                float d = std::sqrt(d2); 
                                float s = 0.5 * (moveDist - d) / d;
                                dx *= s;
                                dy *= s;
                                this->positions[2 * particleIndex] -= dx;
                                this->positions[2 * particleIndex + 1] -= dy;
                                this->positions[2 * otherParticleID] += dx;
                                this->positions[2 * otherParticleID + 1] += dy;
                            }
                        }
                    }
                }
            }
        }
    }

    void makeParticleQueries(int index) {
        for (int i = -1; i < 2; ++i) {
            for (int j = -int(std::min(positions[2 * index + 1] / this->spacing, 1.f)); j < int(std::min(std::ceil(this->nY - positions[2 * index + 1] / this->spacing), 2.f)); ++j) {
                int cell = this->getCell(this->positions[index * 2] + i * this->spacing, this->positions[index * 2 + 1] + j * this->spacing);
                if (cell < 0 || cell > tableSize - 1) continue;
                int start = this->cellCount[cell];
                int end = this->cellCount[cell + 1];
                for (int p = start; p < end; ++p) {
                    int otherParticleID = this->particleArray[p];

                    if (otherParticleID == index) continue;

                    /*if (index == 0) {
                        particleColors[otherParticleID * 3] = 0;
                        particleColors[otherParticleID * 3 + 1] = 255;
                        particleColors[otherParticleID * 3 + 2] = 0;
                    }*/

                    float dx = this->positions[otherParticleID * 2] - this->positions[index * 2];
                    float dy = this->positions[otherParticleID * 2 + 1] - this->positions[index * 2 + 1];
                    float d2 = dx * dx + dy * dy;
                    if (d2 > checkSeperationDist || d2 == 0.0) continue;
                    float d = std::sqrt(d2); 
                    float s = 0.5 * (moveDist - d) / d;
                    dx *= s;
                    dy *= s;
                    this->positions[2 * index] -= dx;
                    this->positions[2 * index + 1] -= dy;
                    this->positions[2 * otherParticleID] += dx;
                    this->positions[2 * otherParticleID + 1] += dy;
                }
            }
        }  
    }

    void makeForceObjectQueries(bool forceObjectActive) {
        if (forceObjectActive) {
            // might have to make this std::max(1, ...); 
            float numCovered = forceObjectRadius / (this->spacing);
            if ((int)(numCovered) != numCovered) {
                numCovered++;
            }
            numCovered = (int)(numCovered);
            for (int i = -numCovered; i < numCovered + 1; ++i) {
                for (int j = -int(std::min(mouseY / this->spacing, numCovered)); j < int(std::min(this->nY - mouseY / this->spacing, numCovered + 1)); ++j) {
                    int cell = this->getCell(mouseX + i * this->spacing, mouseY + j * this->spacing);

                    if (cell < 0 || cell > tableSize - 1) continue;
                    
                    int start = this->cellCount[cell];
                    int end = this->cellCount[cell + 1];
                    for (int p = start; p < end; ++p) {
                        int otherParticleID = this->particleArray[p];

                        /*particleColors[3 * otherParticleID] = 0;
                        particleColors[3 * otherParticleID + 1] = 255;
                        particleColors[3 * otherParticleID + 2] = 0;*/

                        float dx = this->positions[otherParticleID * 2] - mouseX;
                        float dy = this->positions[otherParticleID * 2 + 1] - mouseY;
                        float d2 = dx * dx + dy * dy;
                        if (d2 > checkForceObjectSeperationDist || d2 == 0.0) continue;
                        float d = std::sqrt(d2);
                        forceObjectQueries[otherParticleID] = d;
                    }
                }
            }
        }
    }

    void constrainWalls(int i) {
        float velX = positions[2 * i] - oldPositions[2 * i];
        float velY = positions[2 * i + 1] - oldPositions[2 * i + 1];
        if (this->positions[2 * i] - radius < 0) {
            this->positions[2 * i] = radius;
            if (velX < 0) {
                positions[2 * i] = radius;
                oldPositions[2 * i] = radius + velX * restitution;
                frictions[2 * i + 1] = true;
            }
        }
        else if (this->positions[2 * i] + radius > WIDTH) {
            this->positions[2 * i] = WIDTH - radius;
            if (velX > 0) {
                positions[2 * i] = WIDTH - radius;
                oldPositions[2 * i] = WIDTH - radius + velX * restitution;
                frictions[2 * i + 1] = true;
            }
        }
        if (this->positions[2 * i + 1] - radius < 0) {
            this->positions[2 * i + 1] = radius;
            if (velY < 0) {
                positions[2 * i + 1] = radius;
                oldPositions[2 * i + 1] = radius + velY * restitution;
                this->frictions[2 * i] = true;
            }
        }
        else if (this->positions[2 * i + 1] + radius > HEIGHT) {
            this->positions[2 * i + 1] = HEIGHT - radius;
            if (velY > 0) {
                positions[2 * i + 1] = HEIGHT - radius;
                oldPositions[2 * i + 1] = HEIGHT - radius + velY * restitution;
                this->frictions[2 * i] = true;
            }
        }
    }

    void simulate(sf::RenderWindow& window, const float& dt, bool leftMouseDown, bool rightMouseDown, const int subSteps) {
        sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);
        this->mouseX = mouse_pos.x;
        this->mouseY = mouse_pos.y;

        this->dt = dt / subSteps;

        bool mouseDown = leftMouseDown || rightMouseDown;

        for (int i = 0; i < subSteps; ++i) {
            std::fill(begin(affectedByForceObj), end(affectedByForceObj), false);

            this->InitializeHM();

            this->makeForceObjectQueries(mouseDown);

            if (leftMouseDown) {
                this->applyForceObjectForces(250, dt); // pulling, 250
            }
            else if (rightMouseDown) { // dont worry about the forceObjectbeing     stuck in push/pull mode. the boolean in the seperateparticles method    will take care of that 
                this->applyForceObjectForces(-1000, dt); // pushing, -1000
            }

            /*for (int i = 0; i < numParticles; ++i) {
                this->makeParticleQueries2(1, nX - 1);
            }*/

            for (int i = 0; i < numParticles; ++i) {
                this->makeParticleQueries(i);
            }

            for (int i = 0; i < numParticles; ++i) {
                this->updateParticle(i);
            }

            for (int i = 0; i < numParticles; ++i) {
                this->constrainWalls(i);
            }
        }

        this->drawParticles(window);

        if (mouseDown) {
            this->drawForceObject(window);
        }
    } 

    void updateParticle(int i) {
        float xVel = this->positions[2 * i] - this->oldPositions[2 * i];
        float yVel = this->positions[2 * i + 1] - this->oldPositions[2 * i + 1];

        if (affectedByForceObj[i]) {
            xVel *= 0.99;
            yVel *= 0.99;
        }
    
        //if (Math.abs(xVel) < 0.0002 && this->positions[2 * i + 1] + this->radius >= HEIGHT) {
        //  xVel = 0;
        //}
    
        if (this->frictions[2 * i]) {
          xVel *= 1 / (1 + (dt * this->friction));
        }

        if (this->frictions[2 * i + 1]) {
          yVel *= 1 / (1 + (dt * this->friction));
        }
    
        this->oldPositions[2 * i] = this->positions[2 * i];
        this->oldPositions[2 * i + 1] = this->positions[2 * i + 1];

        float damping = 0.f;
    
        this->positions[2 * i] += (xVel + (this->forces[2 * i] - xVel * damping) * (dt * dt));
        this->positions[2 * i + 1] += (yVel + (this->forces[2 * i + 1] - yVel * damping) * (dt * dt));
    
        this->frictions[2 * i] = false;
        this->frictions[2 * i + 1] = false;
        this->forces[2 * i] = 0;
        this->forces[2 * i + 1] = gravity;
    }

    void applyForceObjectForces(float strength, float dt) {
        for (auto [otherParticleID, dist] : forceObjectQueries) {
            float dx = mouseX - positions[2 * otherParticleID];
            float dy = mouseY - positions[2 * otherParticleID + 1];
            float edgeT = dist / forceObjectRadius;
            float centerT = 1 - edgeT;

            float velX = positions[2 * otherParticleID] - oldPositions[2 * otherParticleID];
            float velY = positions[2 * otherParticleID + 1] - oldPositions[2 * otherParticleID + 1];

            forces[2 * otherParticleID] += (dx * strength - velX) * centerT;
            forces[2 * otherParticleID + 1] += (dy * strength - velX) * centerT;

            if (strength > 0) {
                affectedByForceObj[otherParticleID] = true;
            }
        } 
    }

    void drawForceObject(sf::RenderWindow& window) {
        forceObjectDrawer.setPosition(mouseX, mouseY);
        window.draw(forceObjectDrawer); 
        forceObjectQueries.clear();
    }

    void drawParticles(sf::RenderWindow& window) {
        // low of 0 high of like 100 
        for (int i = 0; i < numParticles; ++i) {  
            this->particleDrawer.setPosition(positions[i * 2], positions[i * 2 + 1]);
            float velX = positions[2 * i] - oldPositions[2 * i];
            float velY = positions[2 * i + 1] - oldPositions[2 * i + 1];
            int vel = (int)(velX * velX + velY * velY) / 0.1; 
            if (vel > gradient.size()) {
                this->particleDrawer.setFillColor(sf::Color(gradient[gradient.size() - 1][0], gradient[gradient.size() - 1][1], gradient[gradient.size() - 1][2]));
            }
            else {
                this->particleDrawer.setFillColor(sf::Color(gradient[vel][0], gradient[vel][1], gradient[vel][2]));
            }

            //this->particleDrawer.setFillColor(sf::Color(particleColors[3 * i], particleColors[3 * i + 1], particleColors[3 * i + 2]));

            window.draw(this->particleDrawer);
        }

        /*for (int i = 0; i < numParticles; ++i) {
            particleColors[i * 3] = 255;
            particleColors[i * 3 + 1] = 0;
            particleColors[i * 3 + 2] = 0;
        }*/
    }

    void drawHashNums(sf::RenderWindow& window, sf::Text& text) {
        for (int i = 0; i < WIDTH; i += this->spacing) {
            for (int j = 0; j < HEIGHT; j += this->spacing) {
                text.setPosition(i, j);
                text.setString(std::to_string(this->hashCoords(i, j)));
                window.draw(text);
            }
        }
    }

    void addToForceObjectRadius(float add) {
        if (forceObjectRadius + add > 0) {
            this->forceObjectRadius += add;
            this->forceObjectDrawer.setOrigin(forceObjectRadius, forceObjectRadius);
            this->forceObjectDrawer.setRadius(forceObjectRadius);
            this->checkForceObjectSeperationDist = (this->radius + forceObjectRadius) * (this->radius + forceObjectRadius);
        }
    }
   
};

int main()
{
    int WIDTH = 2000; //2000, 800
    int HEIGHT = 1000; // 800, 1300 
    int numParticles = 800; // 5000
    float restitution = 0.f;
    float gravity = 1500.f; //1500
    float friction = 10;
    float radius = 20;

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Verlet Simulation");

    sf::Font font;
    font.loadFromFile("C:\\Users\\dklos\\vogue\\Vogue.ttf");

    sf::Text text;
    text.setFont(font);
    text.setPosition(10, 10);
    text.setFillColor(sf::Color::White);

    sf::Clock deltaClock;

    window.setFramerateLimit(120);

    int frame = 0;
    int fps = 0;

    float totalDT = 0;
    float numDT = 0;

    bool rightMouseDown = false;
    bool leftMouseDown = false;

    float subSteps = 8;

    // float WIDTH, float HEIGHT, int numParticles, float gravity, float restitution, float friction
    VerletBalls balls = VerletBalls(WIDTH, HEIGHT, numParticles, gravity, restitution, friction, radius);

    while (window.isOpen())
    {
        sf::Time deltaTime = deltaClock.restart();
        float dt = deltaTime.asSeconds();

        /*totalDT += dt;
        numDT++;*/
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Q) {
                    //std::cout << totalDT / numDT;
                    window.close();
                }
            }
            else if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    leftMouseDown = true;
                }
                else if (event.mouseButton.button == sf::Mouse::Right) {
                    rightMouseDown = true;
                }
            }
            else if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    leftMouseDown = false;
                }
                else if (event.mouseButton.button == sf::Mouse::Right) {
                    rightMouseDown = false;
                }
            }
        }

        window.clear();

        balls.simulate(window, dt, leftMouseDown, rightMouseDown, subSteps);

        frame++;
        if (frame == 30) {
            fps = (int)(1.f / dt);
            frame = 0;
        }

        text.setPosition(WIDTH - 70, 10);
        text.setString(std::to_string(fps));
        window.draw(text);

        window.display();
    }


    return 0;
}
