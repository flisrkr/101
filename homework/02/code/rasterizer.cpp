//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static float orientationTest(float xp, float yp, float xa, float ya, float xb, float yb)
{
    return (xp - xa) * (yb - ya) - (yp - ya) * (xb - xa);
}

static bool insideTriangle(float x, float y, const Vector3f *_v)
{
    float pecd_a = orientationTest(
        static_cast<float>(x), static_cast<float>(y),
        _v[0].x(), _v[0].y(),
        _v[1].x(), _v[1].y());

    float pecd_b = orientationTest(
        static_cast<float>(x), static_cast<float>(y),
        _v[1].x(), _v[1].y(),
        _v[2].x(), _v[2].y());

    float pecd_c = orientationTest(
        static_cast<float>(x), static_cast<float>(y),
        _v[2].x(), _v[2].y(),
        _v[0].x(), _v[0].y());

    const float EPSILON = 1e-6f;

    bool all_positive = (pecd_a >= -EPSILON) && (pecd_b >= -EPSILON) && (pecd_c >= -EPSILON);
    bool all_negative = (pecd_a <= EPSILON) && (pecd_b <= EPSILON) && (pecd_c <= EPSILON);

    return all_positive || all_negative;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};
        // Homogeneous division
        for (auto &vec : v)
        {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto &vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    const auto &vertices = t.v;

    float min_x = std::min({vertices[0].x(), vertices[1].x(), vertices[2].x()});
    float max_x = std::max({vertices[0].x(), vertices[1].x(), vertices[2].x()});
    float min_y = std::min({vertices[0].y(), vertices[1].y(), vertices[2].y()});
    float max_y = std::max({vertices[0].y(), vertices[1].y(), vertices[2].y()});

    int bb_top = std::min(std::ceil(max_y), height - 1.f);
    int bb_bottom = std::max(std::floor(min_y), 0.f);
    int bb_left = std::max(std::floor(min_x), 0.f);
    int bb_right = std::min(std::ceil(max_x), width - 1.f);

    for (int x = bb_left; x < bb_right; x++)
    {
        for (int y = bb_bottom; y < bb_top; y++)
        {
            int sample_count = 0;
            bool any_sample_covered = false;
            int pixel_index = get_index(x, y);
            for (auto &offset : sample_offsets)
            {
                float x_sample = x + offset.first;
                float y_sample = y + offset.second;
                if (insideTriangle(x_sample, y_sample, vertices))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x_sample, y_sample, vertices);
                    float z_interpolated = alpha * vertices[0].z() + beta * vertices[1].z() + gamma * vertices[2].z();
                    if (z_interpolated < depth_buf[pixel_index][sample_count].depth)
                    {
                        depth_buf[pixel_index][sample_count].depth = z_interpolated;
                        depth_buf[pixel_index][sample_count].color = t.getColor();
                        any_sample_covered = true;
                    }
                }
                sample_count++;
            }
            sample_count = 0;
            if (any_sample_covered)
            {
                Eigen::Vector3f pixel_color = {0.f, 0.f, 0.f};
                for (auto &offset : sample_offsets)
                {
                    pixel_color += depth_buf[pixel_index][sample_count].color;
                    sample_count++;
                }
                pixel_color /= 4.f;
                Eigen::Vector3f pixel = {static_cast<float>(x), static_cast<float>(y), 1.f};
                set_pixel(pixel, pixel_color);
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::vector<sample>(4, sample{std::numeric_limits<float>::infinity(), Eigen::Vector3f{0, 0, 0}}));
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    for (auto &pixel_samples : depth_buf)
    {
        pixel_samples.resize(4, sample{std::numeric_limits<float>::infinity(), Eigen::Vector3f{0, 0, 0}});
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    // old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}