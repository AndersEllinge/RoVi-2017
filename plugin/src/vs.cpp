#include "vs.h"
#include <rw/math.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders.hpp>

std::vector<double> vs::calcUV(double focal, rw::kinematics::Frame* marker, rw::kinematics::Frame* cam,
                               rw::kinematics::Frame* base, rw::kinematics::State state) {
    // Calculate rBase
    rw::kinematics::FKRange fkMarker(base, marker, state);
    rw::math::Transform3D<> rBaseT = fkMarker.get(state);
    rw::math::Vector3D<> rBase = rBaseT.P();

    // Calculate rCam from rBase
    rw::math::Transform3D<> rCamT = cam->fTf(marker, state);
    rw::math::Vector3D<> rCam = rCamT.P();

    //rw::common::Log::log().info() << "rCam: " << rCam << std::endl;

    // Calculate u and v from rCam
    std::vector<double> UV = {(focal * rCam[0]) / rCam[2], (focal * rCam[1]) / rCam[2]};
    UV[0] = -round(UV[0]);
    UV[1] = -round(UV[1]);
    return UV;
}

rw::math::Jacobian vs::calcImageJacobian(double u, double v, double z, double focal) {
    rw::math::Jacobian imageJ(2, 6);
    imageJ(0,0) = -(focal/z);
    imageJ(0,1) = 0;
    imageJ(0,2) = u/z;
    imageJ(0,3) = (u*v)/focal;
    imageJ(0,4) = -(pow(focal, 2)+pow(u, 2))/focal;
    imageJ(0,5) = v;

    imageJ(1,0) = 0;
    imageJ(1,1) = -(focal/z);
    imageJ(1,2) = v/z;
    imageJ(1,3) = ((focal*focal)+(v*v))/focal;
    imageJ(1,4) = -(u*v)/focal;
    imageJ(1,5) = -u;
    return imageJ;
}

Eigen::Matrix<double, 6, 6> vs::calcS(rw::math::Rotation3D<> camRot) {
    Eigen::Matrix<double, 6, 6> S;
    //rw::common::Log::log().info() << "n: " << camRot << std::endl;

    rw::math::Rotation3D<> rBaseCamTrans = camRot.inverse();

    //rw::common::Log::log().info() << "i: " << rBaseCamTrans << std::endl;

    S << rBaseCamTrans(0,0), rBaseCamTrans(0,1), rBaseCamTrans(0,2), 0, 0, 0,
            rBaseCamTrans(1,0), rBaseCamTrans(1,1), rBaseCamTrans(1,2), 0, 0, 0,
            rBaseCamTrans(2,0), rBaseCamTrans(2,1), rBaseCamTrans(2,2), 0, 0, 0,
            0, 0, 0, rBaseCamTrans(0,0), rBaseCamTrans(0,1), rBaseCamTrans(0,2),
            0, 0, 0, rBaseCamTrans(1,0), rBaseCamTrans(1,1), rBaseCamTrans(1,2),
            0, 0, 0, rBaseCamTrans(2,0), rBaseCamTrans(2,1), rBaseCamTrans(2,2);
    return S;
}

rw::math::Q vs::calcDqFromUV(double u, double v, double du, double dv, double z, double focal,
                             rw::kinematics::Frame *camFrame, rw::models::Device::Ptr device,
                             rw::kinematics::State state)
{
    // Calculate image jacobian
    rw::math::Jacobian imageJ = calcImageJacobian(u, v, z, focal);

    // Calculate S
    rw::kinematics::Frame* base = device->getBase();
    rw::kinematics::FKRange fkCamera(base, camFrame, state);
    rw::math::Transform3D<> baseTcam = fkCamera.get(state);
    Eigen::Matrix<double, 6, 6> S = calcS(baseTcam.R());

    // Calculate J(q)
    rw::math::Jacobian J = device->baseJframe(camFrame, state);

    // Calculate ZImage
    Eigen::Matrix<double, 2, 7> ZImage = imageJ.e() * S * J.e();

    rw::common::Log::log().info() << "ZImage: " << std::endl << ZImage << std::endl;
    rw::common::Log::log().info() << "ZImage transposed: " << std::endl << ZImage.transpose() << std::endl;
    rw::common::Log::log().info() << "ZImage*zImageT: " << std::endl << ZImage*ZImage.transpose() << std::endl;

    // Solve for y
    Eigen::Matrix<double, 2, 1> dUdV(du, dv);
    Eigen::Matrix<double, 2, 1> y = rw::math::LinearAlgebra::pseudoInverse(ZImage*ZImage.transpose()) * dUdV;

    rw::common::Log::log().info() << "dUdV: " << dUdV << std::endl;
    rw::common::Log::log().info() << "ZImageWeird: " << rw::math::LinearAlgebra::pseudoInverse(ZImage*ZImage.transpose()) << std::endl;
    rw::common::Log::log().info() << "y: " << y << std::endl;

    // Choose dq
    rw::common::Log::log().info() << "ZImage: " << std::endl << ZImage << std::endl;
    Eigen::Matrix<double, 7, 1> eQ = ZImage.transpose() * y;
    rw::common::Log::log().info() << "dQ: " << std::endl << eQ << std::endl;
    rw::math::Q dq(eQ);

    return dq;
}
