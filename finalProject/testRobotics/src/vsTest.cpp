#include "vs.h"
#include "vsMult.h"
#include <rw/loaders.hpp>
#include <rw/kinematics.hpp>
#include <fstream>

#define focal 823.0
#define ez 0.5

std::vector<rw::math::Transform3D<>> loadTransforms(std::string file);
rw::math::Q addVelLimits(rw::math::Q dq, double t, rw::models::Device::Ptr device);

int main() {

    /* Test with single point */
    // Init
    std::string motionFile = "MarkerMotionFast.txt";
    double deltaT = 1;
    std::ofstream jointCSV;
    std::ofstream jointMultCSV;
    std::ofstream toolCSV;
    std::ofstream toolMultCSV;
    std::ofstream maxDistCSV;
    maxDistCSV.open("maxDist.csv");
    std::ofstream maxDistMultCSV;
    maxDistMultCSV.open("maxDistMult.csv");
    int maxDist = 0;
    std::vector<int> maxDistMult = {0, 0, 0};
    rw::math::Q currentQ;
    rw::math::Transform3D<> currentTrans;

    // Find wc and state
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../PA10WorkCell/ScenePA10RoVi1.wc.xml");
    rw::kinematics::State state = wc->getDefaultState();

    // Get device, marker and camera
    rw::models::Device::Ptr device = wc->findDevice("PA10");
    rw::kinematics::Frame* marker = wc->findFrame("Marker");
    rw::kinematics::Frame* cam = wc->findFrame("Camera");
    rw::kinematics::MovableFrame* mFrame = dynamic_cast<rw::kinematics::MovableFrame*>(marker);
    rw::math::Transform3D<> mFrameReset = mFrame->getTransform(state);
    // Save marker pose init
    currentTrans = wc->getWorldFrame()->fTf(cam, state);
    toolCSV << currentTrans.P()[0] << "," << currentTrans.P()[1] << "," << currentTrans.P()[2] << ","
            << rw::math::RPY<>(currentTrans.R())[0] << "," << rw::math::RPY<>(currentTrans.R())[1] << ","
            << rw::math::RPY<>(currentTrans.R())[2] << std::endl;

    // Init the position of the device
    rw::math::Q init(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
    device->setQ(init, state);
    // Save device q init
    currentQ = device->getQ(state);
    jointCSV << currentQ[0] << "," << currentQ[1] << "," << currentQ[2] << "," << currentQ[3] << ","
             << currentQ[4] << "," << currentQ[5] << "," << currentQ[6] << std::endl;

    // Calc starting UV
    std::vector<double> UV = vs::calcUV(focal, marker, cam, state);

    //std::cout << UV[0] << ", " << UV[1] << std::endl;

    // Load transforms
    std::vector<rw::math::Transform3D<>> transformsForMarker = loadTransforms(motionFile);

    // Calculate motion while moving the marker
    while (deltaT > 0.01) {
        jointCSV.open("jointValues" + std::to_string(deltaT) + ".csv");
        toolCSV.open("toolPose" + std::to_string(deltaT) + ".csv");

        for (int i = 0; i < transformsForMarker.size(); i++) {
            mFrame->setTransform(transformsForMarker[i], state);
            std::vector<double> newUV = vs::calcUV(focal, marker, cam, state);
            std::vector<double> dUV = {newUV[0] - UV[0], newUV[1] - UV[1]};
            rw::math::Q dq = vs::calcDqFromUV(UV[0], UV[1], dUV[0], dUV[1], ez, focal, cam, device, state);
            dq = addVelLimits(dq, deltaT, device);
            rw::math::Q newQ = device->getQ(state) + dq;
            device->setQ(newQ, state);

            // Save q values for plotting
            currentQ = device->getQ(state);
            jointCSV << currentQ[0] << "," << currentQ[1] << "," << currentQ[2] << "," << currentQ[3] << ","
                     << currentQ[4] << "," << currentQ[5] << "," << currentQ[6] << std::endl;

            // Save tool pose for plotting
            currentTrans = wc->getWorldFrame()->fTf(cam, state);
            toolCSV << currentTrans.P()[0] << "," << currentTrans.P()[1] << "," << currentTrans.P()[2] << ","
                    << rw::math::RPY<>(currentTrans.R())[0] << "," << rw::math::RPY<>(currentTrans.R())[1] << ","
                    << rw::math::RPY<>(currentTrans.R())[2] << std::endl;

            std::vector<double> resultUV = vs::calcUV(focal, marker, cam, state);
            std::vector<double> precision = {resultUV[0] - UV[0], resultUV[1] - UV[1]};

            //std::cout << "Precision in U: " << precision[0] << std::endl;
            //std::cout << "Precision in V: " << precision[1] << std::endl << std::endl;

            // Calc new max distance
            int newMaxDist = abs(precision[0]) + abs(precision[1]);
            if (newMaxDist > maxDist)
                maxDist = newMaxDist;
        }

        maxDistCSV << deltaT << "," << maxDist << std::endl;
        maxDist = 0;

        // Reset marker frame
        mFrame->setTransform(mFrameReset, state);

        // Init the position of the device
        device->setQ(init, state);

        // Calculate resulting precision
        //std::vector<double> resultUV = vs::calcUV(focal, marker, cam, state);
        //std::vector<double> precision = {resultUV[0] - UV[0], resultUV[1] - UV[1]};

        //std::cout << "Final precision in U: " << precision[0] << std::endl;
        //std::cout << "Final precision in V: " << precision[1] << std::endl;

        jointCSV.close();
        toolCSV.close();

        deltaT = deltaT - 0.05;
    }

    maxDistCSV.close();
    deltaT = 1.0;

    /* Test with multiple points */
    rw::kinematics::Frame* p1 = wc->findFrame("P1");
    rw::kinematics::Frame* p2 = wc->findFrame("P2");
    rw::kinematics::Frame* p3 = wc->findFrame("P3");

    // Reset marker frame
    mFrame->setTransform(mFrameReset, state);

    // Init the position of the device
    device->setQ(init, state);

    // Calc starting UVs
    std::vector<std::vector<double>> UVs = vsMult::calcUVMult(focal, p1, p2, p3, cam, state);
    //std::cout << "size: " << UVs[0].size() << std::endl;
    for (int k = 0; k < UVs[0].size(); ++k) {
        //std::cout << UVs[0][k] << ", " << UVs[1][k] << std::endl;
    }

    while (deltaT > 0.01) {
        jointMultCSV.open("jointValuesMult" + std::to_string(deltaT) + ".csv");
        toolMultCSV.open("toolPoseMult" + std::to_string(deltaT) + ".csv");

        // Calculate motion while moving the marker
        for (int i = 0; i < transformsForMarker.size(); i++) {
            mFrame->setTransform(transformsForMarker[i], state);
            std::vector<std::vector<double>> newUVs = vsMult::calcUVMult(focal, p1, p2, p3, cam, state);
            //std::cout << "size: " << newUVs[0].size() << std::endl;
            std::vector<std::vector<double>> dUVs = vsMult::calcDuDv(UVs, newUVs);
            //std::cout << "size: " << dUVs[0].size() << std::endl;
            for (int j = 0; j < dUVs[0].size(); ++j) {
            std::cout << "dU for point " << j << ": " << dUVs[0][j] << std::endl;
            std::cout << "dV for point " << j << ": " << dUVs[1][j] << std::endl;
            }

            rw::math::Q dq = vsMult::calcDqFromUV(UVs[0], UVs[1], dUVs[0], dUVs[1], ez, focal, cam, device, state);
            std::cout << "dq: " << dq << std::endl;
            //dq = addVelLimits(dq, deltaT, device);
            //std::cout << "dq vel: " << dq << std::endl;
            rw::math::Q newQ = device->getQ(state) + dq;
            //std::cout << "oldQ: " << device->getQ(state) << std::endl;
            //std::cout << "newQ: " << newQ << std::endl;
            device->setQ(newQ, state);
            //std::cout << "newCurrentQ: " << device->getQ(state) << std::endl;

            // Save q values for plotting
            currentQ = device->getQ(state);
            jointMultCSV << currentQ[0] << "," << currentQ[1] << "," << currentQ[2] << "," << currentQ[3] << ","
                     << currentQ[4] << "," << currentQ[5] << "," << currentQ[6] << std::endl;

            // Save tool pose for plotting
            currentTrans = wc->getWorldFrame()->fTf(cam, state);
            toolMultCSV << currentTrans.P()[0] << "," << currentTrans.P()[1] << "," << currentTrans.P()[2] << ","
                    << rw::math::RPY<>(currentTrans.R())[0] << "," << rw::math::RPY<>(currentTrans.R())[1] << ","
                    << rw::math::RPY<>(currentTrans.R())[2] << std::endl;

            std::vector<std::vector<double>> resultUVs = vsMult::calcUVMult(focal, p1, p2, p3, cam, state);
            std::vector<std::vector<double>> precision = vsMult::calcDuDv(UVs, resultUVs);
            //std::cout << "size: " << precision[0].size() << std::endl;

            std::vector<int> newMaxDist = {abs(precision[0][0]) + abs(precision[1][0]), abs(precision[0][1]) + abs(precision[1][1]),
                                abs(precision[0][2]) + abs(precision[1][2])};
            if (newMaxDist[0] > maxDistMult[0])
                maxDistMult[0] = newMaxDist[0];
            if (newMaxDist[1] > maxDistMult[1])
                maxDistMult[1] = newMaxDist[1];
            if (newMaxDist[2] > maxDistMult[2])
                maxDistMult[2] = newMaxDist[2];

            for (int j = 0; j < precision[0].size(); j++) {
            std::cout << "Precision in U for point " << j << ": " << precision[0][j] << std::endl;
            std::cout << "Precision in V for point " << j << ": " << precision[1][j] << std::endl;
            }

            //std::cout << std::endl;
        }

        maxDistMultCSV << deltaT << "," << maxDistMult[0] << "," << maxDistMult[1] << "," << maxDistMult[2] << std::endl;
        maxDistMult[0] = 0;
        maxDistMult[1] = 0;
        maxDistMult[2] = 0;

        // Reset marker frame
        mFrame->setTransform(mFrameReset, state);

        // Init the position of the device
        device->setQ(init, state);

        // Calculate final precision
        //std::vector<std::vector<double>> fresultUVs = vsMult::calcUVMult(focal, p1, p2, p3, cam, state);
        //std::vector<std::vector<double>> fprecision = vsMult::calcDuDv(UVs, fresultUVs);
        //std::cout << "size: " << precision[0].size() << std::endl;

        //for (int j = 0; j < fprecision[0].size(); j++) {
        //std::cout << "Final precision in U for point " << j << ": " << fprecision[0][j] << std::endl;
        //std::cout << "Final precision in V for point " << j << ": " << fprecision[1][j] << std::endl;
        //}

        jointMultCSV.close();
        toolMultCSV.close();

        deltaT = deltaT - 0.05;
    }

    maxDistMultCSV.close();

    return 0;
}

std::vector<rw::math::Transform3D<>> loadTransforms(std::string file) {
    // Init
    std::vector<rw::math::Transform3D<>> trans;
    std::ifstream f("../motions/" + file);

    // Stream and create transforms
    double x,y,z,r,p,q;
    while(f >> x >> y >> z >> r >> p >> q) {
        rw::math::Transform3D<> tmp(rw::math::Vector3D<>(x,y,z), rw::math::RPY<>(r,p,q).toRotation3D());
        trans.push_back(tmp);
    }

    /*
    // Test
    for (int i = 0; i < trans.size(); i++) {
        std::cout << trans[i] << std::endl;
    }
    */

    return trans;

}

rw::math::Q addVelLimits(rw::math::Q dq, double t, rw::models::Device::Ptr device) {
    rw::math::Q ndq(7);
    rw::math::Q maxQ = device->getVelocityLimits();

    if (t < 0.0) {
        for (int i = 0; i < ndq.size(); ++i) {
            ndq[i] = 0;
        }
        return ndq;
    }

    for (int i = 0; i < ndq.size(); ++i) {
        if ((fabs(dq[i]) / t) >= maxQ[i]) {
            //std::cout << "vel lim: " << maxQ << std::endl;
            //std::cout << "dq / t:  " << dq / t << std::endl;
            if (dq[i] > 0)
                ndq[i] = (maxQ[i] * t);
            else
                ndq[i] = -(maxQ[i] * t);
        }
        else {
            ndq[i] = dq[i];
        }
    }

    //std::cout << "ndq: " << ndq << std::endl;
    return ndq;
}