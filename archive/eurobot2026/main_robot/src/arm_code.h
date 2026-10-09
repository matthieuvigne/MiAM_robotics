#ifndef ARM_IK_H
#define ARM_IK_H

    #include <eigen3/Eigen/Dense>

    double constexpr l1 = 85.5e-3;   // [m]
    double constexpr l2 = 95.5e-3;   // [m]
    double constexpr l3 = 61.0e-3;   // [m]

    // Solve IK problem
    // pos : x, z, angle
    // initialAngle: initial servo angles.
    Eigen::Vector3d solveArmPosition(Eigen::Vector3d const pos, Eigen::Vector3d const initialAngle);

#endif
