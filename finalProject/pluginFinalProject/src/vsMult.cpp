#include "vsMult.h"
#include "vs.h"
#include <rw/math.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders.hpp>

std::vector<std::vector<double>> vsMult::calcUVMult(double focal, rw::kinematics::Frame *p1, rw::kinematics::Frame *p2,
                                                rw::kinematics::Frame *p3, rw::kinematics::Frame *cam,
                                                rw::kinematics::State state) {
    // Init
    std::vector<double> U;
    std::vector<double> V;
    std::vector<std::vector<double>> UV;

    // Calc point 1
    rw::math::Transform3D<> rCamTp1 = cam->fTf(p1, state);
    rw::math::Vector3D<> rCamP1 = rCamTp1.P();
    //std::vector<double> UVp1 = {(focal * rCamP1[0]) / rCamP1[2], (focal * rCamP1[1]) / rCamP1[2]};
    //UVp1[0] = -round(UVp1[0]);
    //UVp1[1] = -round(UVp1[1]);

    U.push_back(-round((focal * rCamP1[0]) / rCamP1[2]));
    V.push_back(-round((focal * rCamP1[1]) / rCamP1[2]));

    // Calc point 2
    rw::math::Transform3D<> rCamTp2 = cam->fTf(p2, state);
    rw::math::Vector3D<> rCamP2 = rCamTp2.P();
    //std::vector<double> UVp2 = {(focal * rCamP2[0]) / rCamP2[2], (focal * rCamP2[1]) / rCamP2[2]};
    //UVp2[0] = -round(UVp2[0]);
    //UVp2[1] = -round(UVp2[1]);

    U.push_back(-round((focal * rCamP2[0]) / rCamP2[2]));
    V.push_back(-round((focal * rCamP2[1]) / rCamP2[2]));

    // Calc point 3
    rw::math::Transform3D<> rCamTp3 = cam->fTf(p3, state);
    rw::math::Vector3D<> rCamP3 = rCamTp3.P();
    //std::vector<double> UVp3 = {(focal * rCamP3[0]) / rCamP3[2], (focal * rCamP3[1]) / rCamP3[2]};
    //UVp3[0] = -round(UVp3[0]);
    //UVp3[1] = -round(UVp3[1]);

    U.push_back(-round((focal * rCamP3[0]) / rCamP3[2]));
    V.push_back(-round((focal * rCamP3[1]) / rCamP3[2]));

    UV.push_back(U);
    UV.push_back(V);

    //std::cout << "size: " << U.size() << std::endl;
    //std::cout << "size: " << V.size() << std::endl;

    //std::cout << "size: " << UV[0].size() << std::endl;
    //std::cout << "size: " << UV[1].size() << std::endl;

    return UV;
}

rw::math::Jacobian vsMult::calcImageJacobian(std::vector<double> u, std::vector<double> v, double z, double focal) {

    if (u.size() != v.size())
        return rw::math::Jacobian(0, 0);

    rw::math::Jacobian imageJ(2*u.size(), 6);

    for (int i = 0; i < u.size(); i++) {
        imageJ(i*2,0) = -(focal/z);
        imageJ(i*2,1) = 0;
        imageJ(i*2,2) = u[i]/z;
        imageJ(i*2,3) = (u[i]*v[i])/focal;
        imageJ(i*2,4) = -(pow(focal, 2)+pow(u[i], 2))/focal;
        imageJ(i*2,5) = v[i];

        imageJ((i*2)+1,0) = 0;
        imageJ((i*2)+1,1) = -(focal/z);
        imageJ((i*2)+1,2) = v[i]/z;
        imageJ((i*2)+1,3) = ((focal*focal)+(v[i]*v[i]))/focal;
        imageJ((i*2)+1,4) = -(u[i]*v[i])/focal;
        imageJ((i*2)+1,5) = -u[i];
    }

    return imageJ;
}

rw::math::Q
vsMult::calcDqFromUV(std::vector<double> u, std::vector<double> v, std::vector<double> du, std::vector<double> dv,
                     double z, double focal, rw::kinematics::Frame *camFrame,
                     rw::models::Device::Ptr device, rw::kinematics::State state) {

    // Calculate image jacobian
    rw::math::Jacobian imageJ = calcImageJacobian(u, v, z, focal);

    // Calculate S
    rw::kinematics::Frame* base = device->getBase();
    rw::kinematics::FKRange fkCamera(base, camFrame, state);
    rw::math::Transform3D<> baseTcam = fkCamera.get(state);
    Eigen::Matrix<double, 6, 6> S = vs::calcS(baseTcam.R());

    // Calculate J(q)
    rw::math::Jacobian J = device->baseJframe(camFrame, state);

    // Calculate ZImage
    Eigen::MatrixXd ZImage;
    ZImage = imageJ.e() * S * J.e();

    //rw::common::Log::log().info() << "ZImage: " << std::endl << ZImage << std::endl;
    //rw::common::Log::log().info() << "ZImage transposed: " << std::endl << ZImage.transpose() << std::endl;
    //rw::common::Log::log().info() << "ZImage*zImageT: " << std::endl << ZImage*ZImage.transpose() << std::endl;

    // Solve for y
    Eigen::MatrixXd dUdV(du.size()*2, 1);
    for (int i = 0; i < du.size(); i++) {
        dUdV(i*2, 0) = du[i];
        dUdV((i*2)+1, 0) = dv[i];
    }
    //rw::common::Log::log().info() << "dUdV: " << std::endl << dUdV << std::endl;
    Eigen::MatrixXd y;
    y = rw::math::LinearAlgebra::pseudoInverse(ZImage*ZImage.transpose()) * dUdV;

    //rw::common::Log::log().info() << "dUdV: " << dUdV << std::endl;
    //rw::common::Log::log().info() << "ZImageWeird: " << rw::math::LinearAlgebra::pseudoInverse(ZImage*ZImage.transpose()) << std::endl;
    //rw::common::Log::log().info() << "y: " << y << std::endl;

    //SVD METHODS, SEEMS NOT WORK WITH CURRENT CODE
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd1(ZImage, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Eigen::MatrixXd dq1_e = svd1.solve(dUdV);
    //rw::common::Log::log().info() << "SVD dQ: " << std::endl << dq1_e << std::endl;

    // Choose dq
    //rw::common::Log::log().info() << "ZImage: " << std::endl << ZImage << std::endl;
    Eigen::Matrix<double, 7, 1> eQ = ZImage.transpose() * y;
    //rw::common::Log::log().info() << "dQ: " << std::endl << eQ << std::endl;
    rw::math::Q dq(eQ);

    return dq;
}

std::vector<std::vector<double>>
vsMult::calcDuDv(std::vector<std::vector<double>> from, std::vector<std::vector<double>> to) {

    std::vector<double> du;
    std::vector<double> dv;
    std::vector<std::vector<double>> out;

    //std::cout << "size: " << from[0].size() << std::endl;

    // Fix
    for (int i = 0; i < from[0].size(); i++) {
        du.push_back(to[0][i] - from[0][i]);
        dv.push_back(to[1][i] - from[1][i]);
    }

    out.push_back(du);
    out.push_back(dv);

    return out;
}
