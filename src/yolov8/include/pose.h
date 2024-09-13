#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>


class pose
{
    private:
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        Eigen::MatrixXd maCamera_Intrinsics; //  相机内参
        std::vector<Eigen::Vector3d> poses; // 位姿
        Eigen::Matrix3d ellipse_matrix; // 像素坐标系下的椭圆矩阵
        std::vector<Eigen::Matrix3d> ellipse_cone_matrix; // 非标准椭圆锥矩阵
    public:
        pose();
        ~pose();
        Eigen::Vector3d get_psoe(double a1, double a2, double a3, double r = 600.0);
        std::vector<Eigen::Matrix3d> get_Q(cv::Mat& mask, cv::Mat& img); // 获取图片中所有圆在相机坐标系下的非标准椭圆锥矩阵
        std::vector<Eigen::Vector3d> standardization(std::vector<Eigen::Matrix3d> matrixs); // 标准化椭圆锥矩阵：获取标准椭圆锥矩阵方程的参数和对应的旋转矩阵

};
