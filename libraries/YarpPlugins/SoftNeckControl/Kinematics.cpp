// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath>

#include <Eigen/Dense>

using namespace humasoft;

// -----------------------------------------------------------------------------

void SoftNeckControl::computeIk(double theta, double phi, std::vector<double> & lengths)
{
    if (std::abs(theta) < 1e-6)
    {
        theta = 0.001 * M_PI / 180.0;
    }

    phi -= 30.0 * M_PI / 180.0; // offset
    double factor = std::sqrt(3.0) / 2.0;

    // Matrix A
    Eigen::MatrixXd A(4, 3);

    A <<   0.0, -factor * geomA, factor * geomA,
         geomA,    -0.5 * geomA,   -0.5 * geomA,
           0.0,             0.0,            0.0,
           1.0,             1.0,            1.0;

    // Matrix B
    Eigen::MatrixXd B(4, 3);

    B <<   0.0,  -factor * geomB, factor * geomB,
         geomB,     -0.5 * geomB,   -0.5 * geomB,
           0.0,              0.0,            0.0,
           1.0,              1.0,            1.0;

    // Matrix R
    Eigen::MatrixXd R(3, 3);

    double sphi2 = std::pow(std::sin(phi), 2.0);
    double ctheta2 = std::pow(std::sin(theta), 2.0);

    double t11 = sphi2 + std::cos(theta) * ctheta2;
    double t12 = (std::cos(theta) - 1.0) * std::cos(phi) * std::sin(phi);
    double t21 = t12;
    double t13 = std::sin(theta) * std::cos(phi);
    double t31 = -t13;
    double t23 = std::sin(theta) * std::sin(phi);
    double t32 = -t23;
    double t22 = ctheta2 + std::cos(theta) * sphi2;
    double t33 = std::cos(theta);

    R << t11, t12, t13,
         t21, t22, t23,
         t31, t32, t33;

    // s0 and t0
    double s0 = geomL0 * (1.0 - std::cos(theta)) / theta;
    double t0 = geomL0 * std::sin(theta) / theta;

    // Matrix P traslation
    Eigen::MatrixXd P(3, 1);

    P << s0 * std::cos(phi),
         s0 * std::sin(phi),
                         t0;

    // Matrix T Homogeneous
    Eigen::MatrixXd T(4, 4);

    T << R, P,
         0.0, 0.0, 0.0, 1.0;

    // Matrix length
    Eigen::MatrixXd L(4, 3);
    L = T * B - A;

    // Total Length
    lengths.resize(NUM_ROBOT_JOINTS);
    lengths[0] = geomL0 + geomLg0 - std::sqrt(std::pow(L(0, 2), 2) + std::pow(L(1, 2), 2) + std::pow(L(2, 2), 2)); // L1
    lengths[1] = geomL0 + geomLg0 - std::sqrt(std::pow(L(0, 0), 2) + std::pow(L(1, 0), 2) + std::pow(L(2, 0), 2)); // L2
    lengths[2] = geomL0 + geomLg0 - std::sqrt(std::pow(L(0, 1), 2) + std::pow(L(1, 1), 2) + std::pow(L(2, 1), 2)); // L3
}

// -----------------------------------------------------------------------------
