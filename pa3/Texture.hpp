#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v){
        u = std::fmax(0.0f, std::fmin(1.0f, u));
        v = std::fmax(0.0f, std::fmin(1.0f, v));


        auto u_img = u * width;
        auto v_img = (1.0f - v) * height;

        if (u_img >= width)  u_img = width - 1;
        if (v_img >= height) v_img = height - 1;

        auto color = image_data.at<cv::Vec3b>( (int)v_img, (int)u_img );
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    

};
#endif //RASTERIZER_TEXTURE_H
