#ifndef OUTPUT_PATH
#define OUTPUT_PATH
#endif

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
    {
        return control_points[0];
    }
    std::vector<cv::Point2f> next_control_points;
    float x, y;
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        x = control_points[i].x * t + control_points[i + 1].x * (1.f - t);
        y = control_points[i].y * t + control_points[i + 1].y * (1.f - t);
        next_control_points.emplace_back(x, y);
    }
    return recursive_bezier(next_control_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    int bb_l = window.cols - 1, bb_r = 0, bb_b = window.rows - 1, bb_t = 0;
    for (double t = 0.f; t <= 1.f; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        if (point.x < bb_l)
        {
            bb_l = point.x;
        }
        if (point.x > bb_r)
        {
            bb_r = point.x;
        }
        if (point.y < bb_b)
        {
            bb_b = point.y;
        }
        if (point.y > bb_t)
        {
            bb_t = point.y;
        }
    }
    bb_l = std::max(bb_l - 1, 0);
    bb_r = std::min(bb_r + 1, window.cols - 1);
    bb_b = std::max(bb_b - 1, 0);
    bb_t = std::min(bb_t + 1, window.rows - 1);

    int aa_offsets[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    int surround_curve_pixel = 0;
    int near_pixel_x, near_pixel_y;

    for (int i = bb_l; i <= bb_r; i++)
    {
        for (int j = bb_b; j <= bb_t; j++)
        {
            if (window.at<cv::Vec3b>(j, i)[1] == 255)
            {
                continue;
            }
            else
            {
                surround_curve_pixel = 0;
                for (int k = 0; k < 4; k++)
                {
                    near_pixel_x = i + aa_offsets[k][0];
                    near_pixel_y = j + aa_offsets[k][1];
                    if (window.at<cv::Vec3b>(near_pixel_y, near_pixel_x)[1] == 255 &&
                        window.at<cv::Vec3b>(near_pixel_y, near_pixel_x)[0] == 0 &&
                        window.at<cv::Vec3b>(near_pixel_y, near_pixel_x)[2] == 0)
                    {
                        surround_curve_pixel++;
                    }
                }
                window.at<cv::Vec3b>(j, i)[1] = surround_curve_pixel / 4.f * 255;
            }
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite(std::string(OUTPUT_PATH) + "/my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
