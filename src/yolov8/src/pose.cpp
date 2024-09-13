#include "pose.h"
#include "utils.h"


pose::~pose(){}

// 初始化相机内参 : maCamera_Intrinsics(3, 4)
pose::pose()
{
    maCamera_Intrinsics << 320, 0,  320, 0,
                            0, 320, 240, 0,
                            0,  0,   1,  0;
}


// center of circle 3D
Eigen::Vector3d pose::get_psoe(double a1, double a2, double a3, double r)
{
    // 解算存在二义性，但圆心位姿解算出来相同
    Eigen::Vector3d pose1, pose2;
    pose1 << r * sqrt((std::abs(a3)*(std::abs(a1)-std::abs(a2)))/(std::abs(a1)*(std::abs(a1)+std::abs(a3)))), 0, 
             r * sqrt((std::abs(a1)*(std::abs(a2)+std::abs(a3)))/(std::abs(a3)*(std::abs(a1)+std::abs(a3))));
    
    pose2 << -r * sqrt((std::abs(a3)*(std::abs(a1)-std::abs(a2)))/(std::abs(a1)*(std::abs(a1)+std::abs(a3)))), 0, 
              r * sqrt((std::abs(a1)*(std::abs(a2)+std::abs(a3)))/(std::abs(a3)*(std::abs(a1)+std::abs(a3))));

    return pose1;
}



std::vector<Eigen::Matrix3d> pose::get_Q(cv::Mat& mask, cv::Mat& img)
{
    cv::Mat src_binary;
    ellipse_cone_matrix.clear();
	cv::Canny(mask, src_binary, 80, 160);
    // cv::imshow("Ellipse", src_binary);
    cv::findContours(src_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

    for (int i = 0; i < contours.size(); i++)
	{
		if(contours[i].size() > 6)
		{
			cv::RotatedRect ellipse_rect = cv::fitEllipse(contours[i]);		
			//获取椭圆的长轴和短轴长度(外接矩形的长和宽)
			int w = ellipse_rect.size.width;
			int h = ellipse_rect.size.height;
			//椭圆的中心坐标
			cv::Point center = ellipse_rect.center;
			//绘制拟合椭圆
			cv::ellipse(img, ellipse_rect, cv::Scalar(0, 0, 255), 1, 8);
            // std::cout<<"椭圆长轴: "<<ellipse_rect.size.width<<" 椭圆短轴: "<<ellipse_rect.size.height<<" 圆心："<<center<<std::endl;
            
            double cx=ellipse_rect.center.x,cy=ellipse_rect.center.y,a=ellipse_rect.size.width/2,b=ellipse_rect.size.height/2,rot_ang=ellipse_rect.angle*M_PI/180.0;
            double A,B,C,D,E,F;

            A=(a*sin(rot_ang))*(a*sin(rot_ang))+(b*cos(rot_ang))*((b*cos(rot_ang)));
            B=-2*(a*a-b*b)*sin(rot_ang)*cos(rot_ang);
            C=(a*cos(rot_ang))*(a*cos(rot_ang))+(b*sin(rot_ang))*(b*sin(rot_ang));
            D=-(2*A*cx+B*cy);
            E=-(2*C*cy+B*cx);
            F=A*cx*cx+B*cx*cy+C*cy*cy-(a*b)*(a*b);
            
            // 输出椭圆方程系数
            // std::cout<<"A: "<<B/A<<",B: "<<C/A<<",C: "<<D/A<<",D: "<<E/A<<",E: "<<F/A<<std::endl;

            //像素坐标系下的椭圆矩阵: 
            ellipse_matrix << 1,    B/A/2, D/A/2,
                      B/A/2,  C/A,  E/A/2,
                      D/A/2, E/A/2,  F/A;
            // 椭圆锥矩阵
            Eigen::MatrixXd elliptical_cone = maCamera_Intrinsics.transpose() * ellipse_matrix * maCamera_Intrinsics;
            // std::cout <<"椭圆锥矩阵："<<std::endl;
            // std::cout <<elliptical_cone << std::endl;
            // std::cout <<"简化椭圆锥矩阵："<<std::endl; 
            // 简化椭圆锥矩阵
            Eigen::Matrix3d Q = elliptical_cone.block(0, 0, 3, 3);
            // std::cout <<Q<< std::endl;
            ellipse_cone_matrix.push_back(Q);
        }
    }

    return ellipse_cone_matrix;
}

std::vector<Eigen::Vector3d> pose::standardization(std::vector<Eigen::Matrix3d> matrixs)
{
    //////////////
    // 相机坐标系转换到标准空间计算 正交变换
    // 对 Q 进行特征值分解

    for(int i=0; i < matrixs.size(); i++)
    {

        Eigen::Matrix3d Q = matrixs[i];
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(Q);

        // 获取特征向量矩阵P和特征值矩阵H
        Eigen::MatrixXd P = solver.eigenvectors().real();
        Eigen::VectorXd H = solver.eigenvalues().real();

        // 验证 P^T * Q * P 是否为对角矩阵
        // Eigen::MatrixXd verification = P.transpose() * Q * P;

        // std::cout << "特征向量矩阵 P: " << std::endl << P << std::endl;
        // std::cout << "特征值矩阵 H: " << std::endl << H << std::endl;
        // std::cout << "验证 P^T * Q * P 是否为对角矩阵：" << std::endl << verification << std::endl;


        double a1, a2, a3;
        // double u1 = H[0];
        // double u2 = H[1];
        // double u3 = H[2];
        // Eigen::Vector3d e1_cross, e2_cross, e3_cross;
        Eigen::MatrixXd P_(3, 3);
        bool flag;

        // 确定标准椭圆锥的a1, a2, a3:(a1X^2+a2Y^2+a3Z^2=0)
        if ((H[0] * H[1] > 0) || (H[0] *  H[2] > 0) || (H[1] * H[2] > 0)) 
        {
            // 根据标准椭圆锥方程至少有两个数同号
            if (H[0] * H[1] > 0) {
                a1 = H[0];
                a2 = H[1];
                flag = test(a1, a2);
                a3 = H[2];
                P_.col(2) = P.col(2);
                if (flag){
                    P_.col(0) = P.col(1);
                    P_.col(1) = P.col(0);
                }else{
                    P_.col(0) = P.col(0);
                    P_.col(1) = P.col(1);
                }
            } else if (H[0] *  H[2] > 0) {
                a1 = H[0];
                a2 = H[2];
                flag = test(a1, a2);
                a3 = H[1];
                P_.col(2) = P.col(1);
                if (flag){
                    P_.col(0) = P.col(2);
                    P_.col(1) = P.col(0);
                }else{
                    P_.col(0) = P.col(0);
                    P_.col(1) = P.col(2);
                }
            } else {
                a1 = H[1];
                a2 = H[2];
                flag = test(a1, a2);
                a3 = H[0];
                P_.col(2) = P.col(0);
                if (flag){
                    P_.col(0) = P.col(2);
                    P_.col(1) = P.col(1);
                }else{
                    P_.col(0) = P.col(1);
                    P_.col(1) = P.col(2);
                }
            }

        }

        // std::cout << "a1 = " << a1 << ", a2 = " << a2 << ", a3 = " << a3 << std::endl;
        // 打印结果
        // std::cout << "P_: " <<std::endl<< P_ << std::endl;
        // 标准椭圆锥的圆心位置
        Eigen::Vector3d pose = get_psoe(a1, a2, a3);
        // 相机坐标系下的圆心位置
        Eigen::Vector3d pose_ = P_ * pose;
        // std::cout<<"相机坐标系下的圆心位置: "<<std::endl<<pose_<<std::endl;
        poses.push_back(pose_);
        // 排序 以相机坐标系下的Z轴从小达到排序(圆环越近，Z轴越小)
        std::sort(poses.begin(), poses.end(), sortByThirdElement);

    }

    return poses;
}
