#ifndef MODELS_PATH
#define MODELS_PATH "E:/FLiSrKr/01_Workbench/GAMES/101/homework/07/code/models"
#endif

#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char **argv)
{

    // Change the definition here to change resolution
    Scene scene(1024, 1024);
    // https://github.com/ysj1173886760/Learning/blob/master/graphics/GAMES101/PA7/Assignment7/main.cpp
    Material *light = new Material(MICROFACET,
                                   (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
                                    15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
                                    18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    Material *red_plastic = new Material(MICROFACET, Vector3f(0.f), 0.8, 0, Vector3f(1.0f, 0.05f, 0.04f));
    Material *white_plastic = new Material(MICROFACET, Vector3f(0.f), 0.8, 0, Vector3f(0.875f, 0.81f, 0.78f));
    Material *white_marble = new Material(MICROFACET, Vector3f(0.f), 0.001, 0, Vector3f(0.875f, 0.83f, 0.82f));
    Material *green_plastic = new Material(MICROFACET, Vector3f(0.f), 0.8, 0, Vector3f(0.14f, 1.0f, 0.091f));
    Material *copper = new Material(MICROFACET, Vector3f(0.f), 0.1, 1.0, Vector3f(0.95f, 0.64f, 0.54f));
    Material *silver = new Material(MICROFACET, Vector3f(0.f), 0.01, 1.0, Vector3f(0.95f, 0.93f, 0.88f));
    Material *gold = new Material(MICROFACET, Vector3f(0.f), 0.0001, 1.0, Vector3f(1.00f, 0.71f, 0.29f));
    Material *test = new Material(MICROFACET, Vector3f(0.f), 0.0001, 1.0, Vector3f(0.95f, 0.95f, 0.95f));

    MeshTriangle floor(std::string(MODELS_PATH) + "/cornellbox/floor.obj", white_marble);
    MeshTriangle shortbox(std::string(MODELS_PATH) + "/cornellbox/shortbox.obj", white_marble);
    MeshTriangle tallbox(std::string(MODELS_PATH) + "/cornellbox/tallbox.obj", silver);
    MeshTriangle left(std::string(MODELS_PATH) + "/cornellbox/left.obj", red_plastic);
    MeshTriangle right(std::string(MODELS_PATH) + "/cornellbox/right.obj", green_plastic);
    MeshTriangle light_(std::string(MODELS_PATH) + "/cornellbox/light.obj", light);
    // MeshTriangle bunny(std::string(MODELS_PATH) + "/bunny/bunny.obj", white);

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    // scene.Add(&bunny);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}