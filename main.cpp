#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <array>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <thread>

#include "thread_pool.hpp"
#include "collision_grid.hpp" 
#include "physics.hpp"

void drawObjects(PhysicSolver& solver, sf::VertexArray& va, const float radius, const uint32_t startIndex, const uint32_t endIndex) {
    for (int index = startIndex; index < endIndex; ++index) {
        int i = index * 4;

        const float px = solver.positions[2 * index];
        const float py = solver.positions[2 * index + 1];

        va[i].position = {px - radius, py - radius};
        va[i + 1].position = {px + radius, py - radius};
        va[i + 2].position = {px + radius, py + radius};
        va[i + 3].position = {px - radius, py + radius};

        sf::Color color = sf::Color(100, 100, 100);

        va[i].color = color;
        va[i + 1].color = color;
        va[i + 2].color = color;
        va[i + 3].color = color;
    }
}

int main() {
    float radius = 5.f;

    float scalingFactor = 2 * radius;

    int32_t scaledWIDTH = 2400;
    int32_t scaledHEIGHT = 1060;

    int32_t WIDTH = static_cast<float>(scaledWIDTH) / scalingFactor;
    int32_t HEIGHT = static_cast<float>(scaledHEIGHT) / scalingFactor;

	//int32_t scaledWIDTH = WIDTH * scalingFactor; // 210
    //int32_t scaledHEIGHT = HEIGHT * scalingFactor; // 100 

    int numParticles = 15000; // 5000
    float restitution = 0.f;
    float gravity = 1500.f; //1500
    float friction = 1000;
    sf::RenderWindow window(sf::VideoMode(scaledWIDTH, scaledHEIGHT), "Verlet Simulation");

    window.setFramerateLimit(120);

    sf::Font font;
    font.loadFromFile("C:\\Users\\dklos\\vogue\\Vogue.ttf");

    sf::Text text;
    text.setFont(font);
    text.setPosition(10, 10);
    text.setFillColor(sf::Color::White);

    sf::Clock deltaClock;

    int frame = 0;
    int fps = 0;

    float totalDT = 0;
    float numDT = 0;

    bool rightMouseDown = false;
    bool leftMouseDown = false;

    float subSteps = 8;

    const uint32_t numThreads = 15;

    tp::ThreadPool thread_pool(numThreads);

    PhysicSolver solver{WIDTH, HEIGHT, scaledWIDTH, scaledHEIGHT, radius, thread_pool};

    sf::VertexArray va{sf::PrimitiveType::Quads};

    sf::Texture texture;

    sf::RenderStates states;

    va.resize(numParticles * 4);
    texture.loadFromFile("white_circle.png");
    auto const texture_size = static_cast<sf::Vector2f>(texture.getSize());
    for (int index = 0; index < numParticles; ++index) {
        int i = 4 * index;
        va[i].texCoords = {0.f, 0.f};
        va[i + 1].texCoords = {texture_size.x, 0.f};
        va[i + 2].texCoords = {texture_size.x, texture_size.y};
        va[i + 3].texCoords = {0.f, texture_size.y};

        sf::Color color = sf::Color::Red;

        va[i].color = color;
        va[i + 1].color = color;
        va[i + 2].color = color;
        va[i + 3].color = color;
    }
    states.texture = &texture;

    while (window.isOpen())
    {
        sf::Time deltaTime = deltaClock.restart();
        float dt = deltaTime.asSeconds();
        //totalDT += dt;
        //numDT++;
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

        if (solver.numParticles < numParticles) {
            for (uint32_t i{20}; i--;) {
                solver.createObject(2.0f * scalingFactor, 10.0f * scalingFactor + 1.1f * i * scalingFactor);
                solver.last_positions[2 * solver.numParticles - 2] -= 0.2f * scalingFactor;
            }
        }

        uint32_t particlesPerThread = 1.f * solver.numParticles / numThreads;

        uint32_t numMissedParticles = solver.numParticles - numThreads * particlesPerThread;

        solver.update(dt);

        for (int i = 0; i < numThreads; ++i) {
            thread_pool.addTask([&, i]() {
                drawObjects(solver, va, radius, i * particlesPerThread, i * particlesPerThread + particlesPerThread);
            });
        }

        drawObjects(solver, va, radius, solver.numParticles - numMissedParticles, solver.numParticles);

        thread_pool.waitForCompletion();

        window.draw(va, states);

        //balls.simulate(window, dt, leftMouseDown, rightMouseDown, subSteps);

        frame++;
        if (frame == 30) {
            fps = (int)(1.f / dt);
            frame = 0;
        }

        text.setPosition(scaledWIDTH - 70, 10);
        text.setString(std::to_string(fps));
        window.draw(text);

        window.display();
        
    }
    return 0;
}
    

