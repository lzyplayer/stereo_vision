
#ifndef TRANSFORM_UTILITY
#define TRANSFORM_UTILITY

#include <Eigen/Dense>
#include <string>
#include <vector>
/**function for conversation**/

using namespace Eigen;
namespace stereo_vision{

inline double angleBetweenVectors(Eigen::Vector3d &a, Eigen::Vector3d &b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

inline double angleBetweenVectors(Eigen::Vector3f &a, Eigen::Vector3f &b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

    Matrix3f euler2rot(const float x_pi, const float y_pi, const float z_pi) {
        Matrix3f m;
        m = AngleAxisf(z_pi, Vector3f::UnitZ())
            * AngleAxisf(y_pi, Vector3f::UnitY())
            * AngleAxisf(x_pi, Vector3f::UnitX());
        return m;
    }
/**
     * station_param
     * [0]: arm joint 1 theta
     * [1]: arm joint 2 theta
     * [2]: arm joint 3 theta
     * [3]: arm joint 4 theta
     * [4]: arm joint 5 theta
     * [5]: arm joint 6 theta
     * [6]: arm joint 7 theta
     * [7]: Translation x M_world_arm
     * [8]: Translation y M_world_arm
     * [9]: Translation z M_world_arm
     * [10]: Rotation z M_world_arm
     * [11]: Rotation y M_world_arm
     * [12]: Rotation x M_world_arm
     * [13]: base footprint ID
     * [14]: currentEndCam "55"-1a  "AA"-2b
     */
    Matrix4f stationParam2rot( const std::vector<std::string> &params ){
        Eigen::Matrix4f trans = Matrix4f::Identity();
        trans(0,3)=stof(params[7]);
        trans(1,3)=stof(params[8]);
        trans(2,3)=stof(params[9]);
        float yaw = stof(params[10]);
        float pitch = stof(params[11]);
        float roll = stof(params[12]);
        //check rad! or degreee!(degree default)
        Matrix3f Rot;
        Rot = AngleAxisf(yaw/180*M_PI, Vector3f::UnitZ())
            * AngleAxisf(pitch/180*M_PI, Vector3f::UnitY())
            * AngleAxisf(roll/180*M_PI, Vector3f::UnitX());
        trans.block(0,0,3,3) = Rot;
    }

Eigen::Vector3d calculate_err_R(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample) {
    Eigen::Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    Eigen::Matrix3d t_a_m = GroundTruth.block(0, 0, 3, 3);
    Eigen::Matrix3d t_b_m = sample.block(0, 0, 3, 3);
    Eigen::Vector3d x_a = t_a_m * x;
    Eigen::Vector3d y_a = t_a_m * y;
    Eigen::Vector3d z_a = t_a_m * z;
    Eigen::Vector3d x_b = t_b_m * x;
    Eigen::Vector3d y_b = t_b_m * y;
    Eigen::Vector3d z_b = t_b_m * z;
    Eigen::Vector3d result(angleBetweenVectors(x_a, x_b) / M_PI * 180, angleBetweenVectors(y_a, y_b) / M_PI * 180,
                           angleBetweenVectors(z_a, z_b) / M_PI * 180);
    return result;
}

Eigen::Vector3d calculate_err_T(const Eigen::Matrix4d &GroundTruth, const Eigen::Matrix4d &sample) {
    Eigen::Vector3d gt_t = GroundTruth.block(0, 3, 3, 1);
    Eigen::Vector3d t = sample.block(0, 3, 3, 1);
    Eigen::Vector3d result = gt_t - t;
    return result;
}

Eigen::Vector3f calculate_err_R(const Eigen::Matrix4f &GroundTruth, const Eigen::Matrix4f &sample) {
    Eigen::Vector3f x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    Eigen::Matrix3f t_a_m = GroundTruth.block(0, 0, 3, 3);
    Eigen::Matrix3f t_b_m = sample.block(0, 0, 3, 3);
    Eigen::Vector3f x_a = t_a_m * x;
    Eigen::Vector3f y_a = t_a_m * y;
    Eigen::Vector3f z_a = t_a_m * z;
    Eigen::Vector3f x_b = t_b_m * x;
    Eigen::Vector3f y_b = t_b_m * y;
    Eigen::Vector3f z_b = t_b_m * z;
    Eigen::Vector3f result(angleBetweenVectors(x_a, x_b) / M_PI * 180, angleBetweenVectors(y_a, y_b) / M_PI * 180,
                           angleBetweenVectors(z_a, z_b) / M_PI * 180);
    return result;
}

Eigen::Vector3f calculate_err_T(const Eigen::Matrix4f &GroundTruth, const Eigen::Matrix4f &sample) {
    Eigen::Vector3f gt_t = GroundTruth.block(0, 3, 3, 1);
    Eigen::Vector3f t = sample.block(0, 3, 3, 1);
    Eigen::Vector3f result = gt_t - t;
    return result;
}

void print_err(const Eigen::Matrix4f &GroundTruth, const Eigen::Matrix4f &sample) {
    Eigen::Vector3f rot_err = calculate_err_R(GroundTruth, sample);
    Eigen::Vector3f trans_err = calculate_err_T(GroundTruth, sample);
    //print independently
//    std::cout<<"r_err: dx: "<<rot_err[0]<<"\tdy: "<<rot_err[1]<<"\tdz: "<<rot_err[2]<<"\tt_err: x: "<<trans_err[0]<<"\ty: "<<trans_err[1]<<"\tz: "<<trans_err[2]<<std::endl;
    //print all
    std::cout << "r_err: " << sqrt(pow(rot_err[0], 2) + pow(rot_err[1], 2) + pow(rot_err[2], 2)) << "\tt_err: "
              << sqrt(pow(trans_err[0], 2) + pow(trans_err[1], 2) + pow(trans_err[2], 2)) << std::endl;
}


Vector3f rot2euler(const Matrix3f &m) {
    return m.eulerAngles(2, 1, 0);
}

Quaternionf euler2quat(const float xr_pi, const float yp_pi, const float zy_pi) {
    Quaternionf q;
    q = AngleAxisf(xr_pi, Vector3f::UnitX())
        * AngleAxisf(yp_pi, Vector3f::UnitY())
        * AngleAxisf(zy_pi, Vector3f::UnitZ());
    return q;
}

Quaternionf rot2quat(const Eigen::Matrix4f &pose) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    return quat;
}

Vector3f quat2euler(const Quaternionf &quaternionf) {
    return quaternionf.toRotationMatrix().eulerAngles(0, 1, 2);
}

Matrix3f quat2rot(const Quaternionf &quaternionf) {
    return quaternionf.toRotationMatrix();
}
Matrix3d quat2rot(const Quaterniond &quaterniond) {
    return quaterniond.toRotationMatrix();
}

}


#endif //TRANSFORM_UTILITY
