//
// Created by goksu on 2/25/20.
//
#include <thread>
#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.

static std::pair<int, int> dispensePixel(int t, int total_pixels, int num_threads)
{
    int portion = total_pixels / num_threads;
    int index_start = portion * t;
    int index_end = (t == num_threads - 1) ? total_pixels : portion * (t + 1);
    return std::pair<int, int>(index_start, index_end);
}

static void discreteRender(const Scene &scene, int t, int total_pixels, int num_threads, float imageAspectRatio, float scale, std::vector<Vector3f> &framebuffer, Vector3f eye_pos, int spp)
{
    auto [index_start, index_end] = dispensePixel(t, total_pixels, num_threads);
    for (int ind = index_start; ind < index_end; ind++)
    {
        float x = (2 * (ind % scene.width + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
        float y = (1 - 2 * (ind / scene.width + 0.5) / (float)scene.height) * scale;
        Vector3f dir = normalize(Vector3f(-x, y, 1));
        for (int k = 0; k < spp; k++)
        {
            framebuffer[ind] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
        }
        // UpdateProgress((ind / scene.width) / (float)scene.height);
    }
}

void Renderer::Render(const Scene &scene)
{
    int total_pixels = scene.width * scene.height;
    int num_threads = 8;
    int spp = 256;
    std::vector<std::thread> thread_pool;
    std::vector<Vector3f> framebuffer(total_pixels);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    std::cout << "SPP: " << spp << "\n";
    for (int t = 0; t < num_threads; t++)
    {
        thread_pool.emplace_back(discreteRender, std::ref(scene), t, total_pixels, num_threads, imageAspectRatio, scale, std::ref(framebuffer), eye_pos, spp);
    }
    for (auto &thread : thread_pool)
    {
        thread.join();
    }

    // int m = 0;
    // // change the spp value to change sample ammount
    // std::cout << "SPP: " << spp << "\n";
    // for (uint32_t j = 0; j < scene.height; ++j)
    // {
    //     for (uint32_t i = 0; i < scene.width; ++i)
    //     {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         // Vector3f dir = normalize(Vector3f(x, y, -1));
    //         for (int k = 0; k < spp; k++)
    //         {
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    // UpdateProgress(1.f);

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
