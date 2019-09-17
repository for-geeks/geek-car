#ifndef BAIDU_ACU_CPTCONTROL_LEAST_SQURE_LEAST_SQUARE_H
#define BAIDU_ACU_CPTCONTROL_LEAST_SQURE_LEAST_SQUARE_H
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include <vector>
class LineFitter
{
public:
    double v[5] = {0.0};
    double x_max;
    double x_min;
    void least_square(std::vector<std::pair<float, float>> vec){
        int len = vec.size();
        Eigen::MatrixXd A(len, 5);
        x_min = 9999;
        x_max = 0;
        if (len > 10){
            //0 = ax + bx2 + cx3 + d - y
            for (int i = 0; i < len; i++)
            {
                if (vec.at(i).first < x_min){
                    x_min = vec.at(i).first;
                }
                if (vec.at(i).first > x_max){
                    x_max = vec.at(i).first;
                }
                A(i, 2) = 0.1 * vec.at(i).first;
                A(i, 1) = 0.01 * vec.at(i).first * vec.at(i).first;
                A(i, 0) = 0.001 * vec.at(i).first * vec.at(i).first * vec.at(i).first;
                A(i, 3) = 0.1 * 1;
                A(i, 4) = -0.1 * vec.at(i).second;
            }
            //std::cout << "Matrix A" << std::endl;
            //std::cout << A << std::endl;
            Eigen::SelfAdjointEigenSolver < Eigen::Matrix<double, 5, 5> > eigen_sovler(A.transpose() * A);  // 特征值求解器
            //std::cout << "Eigen Values : " << std::endl;
            //std::cout << eigen_sovler.eigenvalues().transpose() << std::endl;    // 输出特征值
            //std::cout << "Eigen Vector :" << std::endl;
            //std::cout << eigen_sovler.eigenvectors() << std::endl;               // 输出特征向量
            v[0] = eigen_sovler.eigenvectors()(0, 0);                  // 提取第一列特征向量
            v[1] = eigen_sovler.eigenvectors()(1, 0);
            v[2] = eigen_sovler.eigenvectors()(2, 0);
            v[3] = eigen_sovler.eigenvectors()(3, 0);
            v[4] = eigen_sovler.eigenvectors()(4, 0);
            v[0] /= 100 * v[4];
            v[1] /= 10 * v[4];
            v[2] /= 1 * v[4];
            v[3] /= v[4];
            v[4] /= v[4];
            std::cout <<"a:"<< v[0] << " " <<"b:"
                << v[1] << "c: " << v[2] << " d: " << v[3] << " e : " << v[4] << std::endl;
        }
    }
    double solve_poly(double x){
        double y = 0;
        y = v[2] * x + v[1] * x * x + v[0] * x * x * x + v[3];
        return y;
    }
    std::vector<std::pair<float, float>> point_generator(void){
        double x = 0;
        double y = 0;
        std::vector<std::pair<float, float>> ret;
        ret.clear();
        if (x_max == 0){
            return ret;
        }
        double dx = (x_max - x_min) / 40;
        x = x_min;
        for (int i = 0; i < 40; i++){
            x += dx;
            y = solve_poly(x);
            ret.push_back(std::make_pair(x, y));
        }
        return ret;
    }
};
#endif
