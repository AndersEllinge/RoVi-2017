#ifndef VS_H
#define VS_H

#include <rw/models.hpp>

class vs {

public:
    static std::vector<double> calcUV(double focal, rw::kinematics::Frame* marker, rw::kinematics::Frame* cam,
                                      rw::kinematics::Frame* base, rw::kinematics::State state);

    static rw::math::Jacobian calcImageJacobian(double u, double v, double z, double focal);

    static Eigen::Matrix<double, 6, 6> calcS(rw::math::Rotation3D<> camRot);

    static rw::math::Q calcDqFromUV(double u, double v, double du, double dv, double z, double focal,
                                    rw::kinematics::Frame* camFrame, rw::models::Device::Ptr device,
                                    rw::kinematics::State state);

};


#endif //VS_H
