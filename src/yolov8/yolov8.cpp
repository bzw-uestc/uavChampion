#include "include/yolov8_.h"
#include "pose.h"
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>


// bool test1(double& a1, double& a2)
// {
//     if (std::abs(a1) < std::abs(a2)) 
//     {
//         std::swap(a1, a2);
//         return true;
//     }

//     return false;
// }

// // center of circle 3D
// Eigen::Vector3d get_psoe1(double a1, double a2, double a3, double r = 600)
// {
//     // 解算存在二义性，但圆心位姿解算出来相同
//     Eigen::Vector3d pose1, pose2;
//     pose1 << r * sqrt((std::abs(a3)*(std::abs(a1)-std::abs(a2)))/(std::abs(a1)*(std::abs(a1)+std::abs(a3)))), 0, 
//              r * sqrt((std::abs(a1)*(std::abs(a2)+std::abs(a3)))/(std::abs(a3)*(std::abs(a1)+std::abs(a3))));
    
//     pose2 << -r * sqrt((std::abs(a3)*(std::abs(a1)-std::abs(a2)))/(std::abs(a1)*(std::abs(a1)+std::abs(a3)))), 0, 
//               r * sqrt((std::abs(a1)*(std::abs(a2)+std::abs(a3)))/(std::abs(a3)*(std::abs(a1)+std::abs(a3))));

//     return pose1;
// }




int main(){

    std::string wts_name = "../best.wts";
    std::string engine_name = "../yolov8s-seg.engine";
    std::string img_name = "../1.png";
    std::string sub_type = "s";
    std::string cuda_post_process = "c";
    std::string labels_filename = "../circle.txt";

    cv::Mat img = cv::imread(img_name);
    std::vector<cv::Rect> rects;
    Eigen::Matrix3d matrix1; // 椭圆矩阵
    Eigen::MatrixXd matrix2(3, 4); //  相机内参
    matrix2 << 320,  0,  320, 0,
                0,  320, 240, 0,
                0,   0,   1,  0;

    yolov8_seg yolov8; // 检测类
    pose circle_pose; // 位姿解算类

    std::cout<<111<<std::endl;
    yolov8.serialize_engine(wts_name, engine_name, sub_type);
    cuda_preprocess_init(kMaxInputImageSize);
    yolov8.prepare_buffer(cuda_post_process);
    

    // 推理
    auto start = std::chrono::system_clock::now();
    cv::Mat mask_ = yolov8.infer(img, cuda_post_process, rects);
    
    // 位置解算
    std::vector<Eigen::Matrix3d> ellipse_cone_matrix =  circle_pose.get_Q(mask_, img);
    // 以Z轴从小到大排序
    std::vector<Eigen::Vector3d> poses = circle_pose.standardization(ellipse_cone_matrix);

    for (int i = 0; i < poses.size(); i++)
    {
        std::cout<<"相机坐标系下圆心位姿：" <<std::endl<<poses[i]<<std::endl;
    }
    
    auto end = std::chrono::system_clock::now();
    std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    cv::imshow("results", img);
    cv::waitKey();

    return 0;

    // std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;

    // // cv::imshow("mask", mask_);
    
    // cv::Mat src_binary;
	// cv::Canny(mask_, src_binary, 80, 160);

    // // cv::imshow("Ellipse", src_binary);

    // // for (int i = 0; i < rects.size(); i++){}

    // std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(src_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

	// for (int i = 0; i < contours.size(); i++)
	// {
	// 	if(contours[i].size() > 6)
	// 	{
	// 		cv::RotatedRect ellipse_rect = cv::fitEllipse(contours[i]);		
	// 		//获取椭圆的长轴和短轴长度(外接矩形的长和宽)
	// 		int w = ellipse_rect.size.width;
	// 		int h = ellipse_rect.size.height;
	// 		//椭圆的中心坐标
	// 		cv::Point center = ellipse_rect.center;
	// 		//绘制拟合椭圆
	// 		cv::ellipse(img, ellipse_rect, cv::Scalar(0, 0, 255), 1, 8);
    //         // std::cout<<"椭圆长轴: "<<ellipse_rect.size.width<<" 椭圆短轴: "<<ellipse_rect.size.height<<" 圆心："<<center<<std::endl;
    //         double cx=ellipse_rect.center.x,cy=ellipse_rect.center.y,a=ellipse_rect.size.width/2,b=ellipse_rect.size.height/2,rot_ang=ellipse_rect.angle*M_PI/180.0;
    //         double A,B,C,D,E,F;
            
    //         A=(a*sin(rot_ang))*(a*sin(rot_ang))+(b*cos(rot_ang))*((b*cos(rot_ang)));
    //         B=-2*(a*a-b*b)*sin(rot_ang)*cos(rot_ang);
    //         C=(a*cos(rot_ang))*(a*cos(rot_ang))+(b*sin(rot_ang))*(b*sin(rot_ang));
    //         D=-(2*A*cx+B*cy);
    //         E=-(2*C*cy+B*cx);
    //         F=A*cx*cx+B*cx*cy+C*cy*cy-(a*b)*(a*b);
            
    //         //输出椭圆方程系数
    //         // std::cout<<"A: "<<B/A<<",B: "<<C/A<<",C: "<<D/A<<",D: "<<E/A<<",E: "<<F/A<<std::endl;
    //         //椭圆矩阵表示形式
    //         matrix1 << 1,    B/A/2, D/A/2,
    //                   B/A/2,  C/A,  E/A/2,
    //                   D/A/2, E/A/2,  F/A;
    //         // 椭圆锥矩阵
    //         Eigen::MatrixXd elliptical_cone = matrix2.transpose() * matrix1 * matrix2;
    //         // std::cout <<"椭圆锥矩阵："<<std::endl;
    //         // std::cout <<elliptical_cone << std::endl;
    //         // std::cout <<"简化椭圆锥矩阵："<<std::endl; 
    //         Eigen::Matrix3d Q = elliptical_cone.block(0, 0, 3, 3);
    //         std::cout <<Q<< std::endl;
            

    //         //////////////
    //         // 相机坐标系转换到标准空间计算 正交变换
    //         // 对 Q 进行特征值分解
    //         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(Q);

    //         // 获取特征向量矩阵P和特征值矩阵H
    //         Eigen::MatrixXd P = solver.eigenvectors().real();
    //         Eigen::VectorXd H = solver.eigenvalues().real();

    //         // 验证 P^T * Q * P 是否为对角矩阵
    //         Eigen::MatrixXd verification = P.transpose() * Q * P;

    //         std::cout << "特征向量矩阵 P: " << std::endl << P << std::endl;
    //         std::cout << "特征值矩阵 H: " << std::endl << H << std::endl;
    //         std::cout << "验证 P^T * Q * P 是否为对角矩阵：" << std::endl << verification << std::endl;

    //         double a1, a2, a3;
    //         double u1 = H[0];
    //         double u2 = H[1];
    //         double u3 = H[2];
    //         Eigen::Vector3d e1_cross, e2_cross, e3_cross;
    //         Eigen::MatrixXd P_(3, 3);
    //         bool flag;

    //         // 确定标准椭圆锥的a1, a2, a3
    //         if ((u1 * u2 > 0) || (u1 * u3 > 0) || (u2 * u3 > 0)) {
    //             // 至少有两个数同号
    //             if (u1 * u2 > 0) {
    //                 a1 = u1;
    //                 a2 = u2;
    //                 flag = test(a1, a2);
    //                 a3 = u3;
    //                 P_.col(2) = P.col(2);
    //                 if (flag){
    //                     P_.col(0) = P.col(1);
    //                     P_.col(1) = P.col(0);
    //                 }else{
    //                     P_.col(0) = P.col(0);
    //                     P_.col(1) = P.col(1);
    //                 }
    //             } else if (u1 * u3 > 0) {
    //                 a1 = u1;
    //                 a2 = u3;
    //                 flag = test(a1, a2);
    //                 a3 = u2;
    //                 P_.col(2) = P.col(1);
    //                 if (flag){
    //                     P_.col(0) = P.col(2);
    //                     P_.col(1) = P.col(0);
    //                 }else{
    //                     P_.col(0) = P.col(0);
    //                     P_.col(1) = P.col(2);
    //                 }
    //             } else {
    //                 a1 = u2;
    //                 a2 = u3;
    //                 flag = test(a1, a2);
    //                 a3 = u1;
    //                 P_.col(2) = P.col(0);
    //                 if (flag){
    //                     P_.col(0) = P.col(2);
    //                     P_.col(1) = P.col(1);
    //                 }else{
    //                     P_.col(0) = P.col(1);
    //                     P_.col(1) = P.col(2);
    //                 }
    //             }
        
    //         }

    //         std::cout << "a1 = " << a1 << ", a2 = " << a2 << ", a3 = " << a3 << std::endl;

    //         // // 确定标准椭圆锥单位旋转矩阵P
    //         // for (int i = 0; i < P.cols(); ++i) {
    //         //     Eigen::VectorXd column = P.col(i);
    //         //     double norm = column.norm();

    //         //     if (norm > 0) {
    //         //         // 避免除以零
    //         //         P.col(i) /= norm;
    //         //     }
    //         // }

    //         // std::cout << "单位化后的正交矩阵 P: " << std::endl << P << std::endl;

    //         // 确定对应标准椭圆锥的旋转矩阵P
    //         // Eigen::RowVectorXd rowVector(3);
    //         // rowVector << 0, 0, 1;
    //         // Eigen::RowVectorXd result = rowVector * P;
    //         // std::cout << "乘积结果：" << result << std::endl;

            

    //         // 初始化e3列向量
    //         // Eigen::VectorXd e3(3);
            
    //         // 判断第三列的值，并赋值给f3
    //         // if (result(2) > 0) {
    //         //     P_.col(2) = P.col(2);
    //         //     e3_cross = P.col(2);
    //         // } else {
    //         //     P_.col(2) = -P.col(2); // 取第三列的相反向量
    //         //     e3_cross = -P.col(2);
    //         // }
            
    //         // P_.col(1) = P.col(1);
    //         // e2_cross = P.col(1);
    //         // // 计算 e1 向量，使其与 e2 和 e3 向量都垂直

    //         // e1_cross = e2_cross.cross(e3_cross);
    //         // P_.col(0) = e1_cross;

    //         //////////////////

    //         // 打印结果
    //         std::cout << "P_: " <<std::endl<< P_ << std::endl;
    //         // 标准椭圆锥的圆心位置
    //         Eigen::Vector3d pose = get_psoe1(a1, a2, a3);
    //         // 相机坐标系下的圆心位置
    //         Eigen::Vector3d pose_ = P_ * pose;
    //         std::cout<<"相机坐标系下的圆心位置: "<<std::endl<<pose_<<std::endl;
    //         std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	// 	}

	// }
}
