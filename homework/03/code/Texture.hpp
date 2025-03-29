//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    float getHeight(float u, float v)
    {
        return getColor(u, v).norm();
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        int u_floor = std::floor(u_img - 0.5f);
        int v_floor = std::floor(v_img - 0.5f);
        float du = u_img - u_floor;
        float dv = v_img - v_floor;

        int u0 = u_floor < 0 ? width + u_floor : u_floor;
        int v0 = v_floor < 0 ? height + v_floor : v_floor;
        int u1 = u_floor + 1 >= width ? u_floor + 1 - width : u_floor + 1;
        int v1 = v_floor + 1 >= height ? v_floor + 1 - height : v_floor + 1;

        auto texel_00 = image_data.at<cv::Vec3b>(v0, u0);
        auto texel_01 = image_data.at<cv::Vec3b>(v1, u0);
        auto texel_10 = image_data.at<cv::Vec3b>(v0, u1);
        auto texel_11 = image_data.at<cv::Vec3b>(v1, u0);

        auto u0_interpolate = Eigen::Vector3f(texel_00[0] * dv + texel_01[0] * (1.f - dv),
                                              texel_00[1] * dv + texel_01[1] * (1.f - dv), texel_00[2] * dv + texel_01[2] * (1.f - dv));
        auto u1_interpolate = Eigen::Vector3f(texel_10[0] * dv + texel_11[0] * (1.f - dv),
                                              texel_10[1] * dv + texel_11[1] * (1.f - dv), texel_10[2] * dv + texel_11[2] * (1.f - dv));

        auto bilinear_interpolate = u0_interpolate * du + u1_interpolate * (1.f - du);
        return bilinear_interpolate;
    }
};
#endif // RASTERIZER_TEXTURE_H
