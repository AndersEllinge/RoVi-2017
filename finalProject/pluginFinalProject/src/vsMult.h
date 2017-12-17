#ifndef VSMULT_H
#define VSMULT_H

#include <rw/models.hpp>

class vsMult {

public:
    static std::vector<std::vector<double>> calcUVMult(double focal, rw::kinematics::Frame *p1,
                                                       rw::kinematics::Frame *p2, rw::kinematics::Frame *p3,
                                                       rw::kinematics::Frame *cam, rw::kinematics::State state);

    static rw::math::Jacobian calcImageJacobian(std::vector<double> u, std::vector<double> v, double z, double focal);

    static rw::math::Q calcDqFromUV(std::vector<double> u, std::vector<double> v, std::vector<double> du,
                                    std::vector<double> dv, double z, double focal,
                                    rw::kinematics::Frame* camFrame, rw::models::Device::Ptr device,
                                    rw::kinematics::State state);

    static std::vector<std::vector<double>> calcDuDv(std::vector<std::vector<double>> from,
                                                     std::vector<std::vector<double>> to);
};


#endif //VS_H
