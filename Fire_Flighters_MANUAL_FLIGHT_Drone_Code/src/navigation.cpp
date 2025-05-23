#include "controller_ref.h"
#include "sensors.h"
#include "myekf_ref.h"
#include "navigation.h"
#include "matrix.h"
#include <cmath>
#include <vector>
#include <random>
#include <Arduino.h>

#define USE_Kss 0

#define DEBUG 0
#define DEBUG1 0
#define DEBUG2 0
#define ALPHA 0.08

Finite_State_Machine Drone(&onboardControl);




// Function to generate normally distributed random numbers
float randn() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<float> dist(0.0, 1.0);
    return dist(gen);
}

void quat2euler2(float* q, float* phi, float* theta, float* psi) {
    // Extract quaternion elements
    float q0 = q[0]; // w (scalar)
    float q1 = q[1]; // x
    float q2 = q[2]; // y
    float q3 = q[3]; // z

    // Roll (phi) - rotation around X-axis
    *phi = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

    // Pitch (theta) - rotation around Y-axis
    float sin_theta = 2.0 * (q0 * q2 - q3 * q1);
    if (fabs(sin_theta) >= 1.0) {
        *theta = copysign(M_PI / 2.0, sin_theta); // Use 90 degrees if out of range
    }
    else {
        *theta = asin(sin_theta);
    }

    // Yaw (psi) - rotation around Z-axis
    *psi = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}


void Fix_Quaternion(float* State) {
    // Extract quaternion components
    float q0 = State[0];
    float q1 = State[1];
    float q2 = State[2];
    float q3 = State[3];

    // Calculate the norm of the quaternion
    float norm_q = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    // Avoid division by zero in case of a very small norm
    if (norm_q > 1e-8) {
        // Renormalize each component
        State[0] = q0 / norm_q;
        State[1] = q1 / norm_q;
        State[2] = q2 / norm_q;
        State[3] = q3 / norm_q;
    }
    else {
        // Handle edge case (set to a default unit quaternion)
        State[0] = 1.0;
        State[1] = 0.0;
        State[2] = 0.0;
        State[3] = 0.0;
    }

    // Flip the sign of the quaternion if the scalar part is negative
    if (State[0] < 0) {
        State[0] = -State[0];
        State[1] = -State[1];
        State[2] = -State[2];
        State[3] = -State[3];
    }
}

void Quaternian_Rotation_Matrix(const float* Qs, float* Rotation_Matrix) {
    float Q0 = Qs[0];
    float Q1 = Qs[1];
    float Q2 = Qs[2];
    float Q3 = Qs[3];

    // Compute the rotation matrix elements (transposed compared to MATLAB)
    Rotation_Matrix[0] = Q0 * Q0 + Q1 * Q1 - Q2 * Q2 - Q3 * Q3;
    Rotation_Matrix[1] = 2 * (Q1 * Q2 - Q0 * Q3);
    Rotation_Matrix[2] = 2 * (Q1 * Q3 + Q0 * Q2);

    Rotation_Matrix[3] = 2 * (Q1 * Q2 + Q0 * Q3);
    Rotation_Matrix[4] = Q0 * Q0 - Q1 * Q1 + Q2 * Q2 - Q3 * Q3;
    Rotation_Matrix[5] = 2 * (Q2 * Q3 - Q0 * Q1);

    Rotation_Matrix[6] = 2 * (Q1 * Q3 - Q0 * Q2);
    Rotation_Matrix[7] = 2 * (Q2 * Q3 + Q0 * Q1);
    Rotation_Matrix[8] = Q0 * Q0 - Q1 * Q1 - Q2 * Q2 + Q3 * Q3;
}



void Correct_IMU_Data(float* IMU_Data, const float* IMU_Position) {
    // Extract Angular Velocity
    float Angular_Vel[3] = { IMU_Data[0], IMU_Data[1], IMU_Data[2] };

    // Extract Acceleration
    float Accel[3] = { IMU_Data[3], IMU_Data[4], IMU_Data[5] };

    // Euler Force (Set to zero)
    float Euler_Force[3] = { 0.0, 0.0, 0.0 };

    // Compute Centrifugal Force: cross(Angular_Vel, cross(Angular_Vel, IMU_Position))
    float temp[3], Centrifugal_Force[3];
    mat_cross(Angular_Vel, IMU_Position, temp);
    mat_cross(Angular_Vel, temp, Centrifugal_Force);

    // Compute Acceleration at CG and store directly into IMU_Data
    for (int i = 0; i < 3; i++)
        IMU_Data[3 + i] = Accel[i] - Euler_Force[i] - Centrifugal_Force[i];
}


void Correct_Motion_Capture_Orientation_Data(float* Orientation_Data) {
    // No correction needed, just apply Fix_Quaternion
    Fix_Quaternion(Orientation_Data);
}

void Correct_Motion_Capture_Position_Data(float* Position_Data,
    const float* Motion_Capture_Sensor_Position,
    const float* Quaternion) {
    // Compute rotation matrix from quaternion
    float T[9]; // 3x3 rotation matrix
    Quaternian_Rotation_Matrix(Quaternion, T);

    // Extract position
    float Position[3] = { Position_Data[0], Position_Data[1], Position_Data[2] };

    // Compute T * Motion_Capture_Sensor_Position (matrix-vector multiplication)
    float T_Sensor_Position[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            T_Sensor_Position[i] += T[i * 3 + j] * Motion_Capture_Sensor_Position[j];

    // Compute corrected position and store in Position_Data
    for (int i = 0; i < 3; i++)
        Position_Data[i] = Position[i] - T_Sensor_Position[i];
}

void Find_Q_Orientation(float* IMU_Sigma_Gyro, float* Q_Orientation) {
    // Calculate average sigma of the gyroscope
    float average_Sigma_Gyro = (IMU_Sigma_Gyro[0] + IMU_Sigma_Gyro[1] + IMU_Sigma_Gyro[2]) / 3.0;

    // Create the Q_Quaternion matrix
    float Q_Quaternion[4][4] = {
        {average_Sigma_Gyro * average_Sigma_Gyro / 4.0, 0.0, 0.0, 0.0},
        {0.0, average_Sigma_Gyro * average_Sigma_Gyro / 4.0, 0.0, 0.0},
        {0.0, 0.0, average_Sigma_Gyro * average_Sigma_Gyro / 4.0, 0.0},
        {0.0, 0.0, 0.0, average_Sigma_Gyro * average_Sigma_Gyro / 4.0}
    };

    // Fill the Q_Orientation matrix
    // 4x4 block at top-left
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Q_Orientation[i * 7 + j] = Q_Quaternion[i][j];
        }
    }

    // 3x3 block at bottom-right (all zeros by default)
    for (int i = 4; i < 7; i++) {
        for (int j = 4; j < 7; j++) {
            Q_Orientation[i * 7 + j] = 0.0;
        }
    }
}

void Find_H_Orientation(float* H_Orientation) {
    // Create the H_Orientation matrix directly
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                H_Orientation[i * 7 + j] = 1.0; // Identity matrix
            }
            else {
                H_Orientation[i * 7 + j] = 0.0;
            }
        }
        for (int j = 4; j < 7; ++j) {
            H_Orientation[i * 7 + j] = 0.0; // Zero matrix
        }
    }
}


void Find_R_Orientation(const float* Positioning_System_Orientation_Sigmas, float* R_Orientation) {
    float Sig_Q0 = Positioning_System_Orientation_Sigmas[0];
    float Sig_Q1 = Positioning_System_Orientation_Sigmas[1];
    float Sig_Q2 = Positioning_System_Orientation_Sigmas[2];
    float Sig_Q3 = Positioning_System_Orientation_Sigmas[3];

    // Fill the R_Orientation matrix
    R_Orientation[0] = std::pow(Sig_Q0, 2);
    R_Orientation[1] = 0.0;
    R_Orientation[2] = 0.0;
    R_Orientation[3] = 0.0;

    R_Orientation[4] = 0.0;
    R_Orientation[5] = std::pow(Sig_Q1, 2);
    R_Orientation[6] = 0.0;
    R_Orientation[7] = 0.0;

    R_Orientation[8] = 0.0;
    R_Orientation[9] = 0.0;
    R_Orientation[10] = std::pow(Sig_Q2, 2);
    R_Orientation[11] = 0.0;

    R_Orientation[12] = 0.0;
    R_Orientation[13] = 0.0;
    R_Orientation[14] = 0.0;
    R_Orientation[15] = std::pow(Sig_Q3, 2);
}

// Function to compute the quaternion difference
void quaternionDifference(const float* Q1, const float* Q2, float* delta_q) {
    float aligned_Q1[4];
    float dot = 0.0;

    // Compute the dot product within the function
    for (int i = 0; i < 4; ++i) {
        dot += Q1[i] * Q2[i];
    }

    // Align signs of quaternions
    if (dot < 0) {
        for (int i = 0; i < 4; ++i) {
            aligned_Q1[i] = -Q1[i];
        }
    }
    else {
        for (int i = 0; i < 4; ++i) {
            aligned_Q1[i] = Q1[i];
        }
    }

    // Compute element-wise difference
    for (int i = 0; i < 4; ++i) {
        delta_q[i] = aligned_Q1[i] - Q2[i];
    }
}

void Correction_Orientation(float* State_Vector_Orientation, float* Covariance_Orientation, float* R_Orientation,
    float* H_Orientation, float* Orientation_Measurment, float* Predicted_Measurment_Orientation) {

    // Transpose H_Orientation (4x7 to 7x4)
    float H_Transpose[7 * 4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 7; ++j) {
            H_Transpose[j * 4 + i] = H_Orientation[i * 7 + j];
        }
    }

    // PHT = Covariance_Orientation * H_Transpose
    float PHT[7 * 4];
    mat_mult(Covariance_Orientation, 7, 7, H_Transpose, 7, 4, PHT);


    // HP = H_Orientation * Covariance_Orientation
    float HP[4 * 7];
    mat_mult(H_Orientation, 4, 7, Covariance_Orientation, 7, 7, HP);


    // HPHT = HP * H_Transpose
    float HPHT[4 * 4];
    mat_mult(HP, 4, 7, H_Transpose, 7, 4, HPHT);


    // HPHTR = HPHT + R_Orientation
    float HPHTR[4 * 4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            HPHTR[i * 4 + j] = HPHT[i * 4 + j] + R_Orientation[i * 4 + j];
        }
    }


    // INV_HPHTR = inv(HPHTR)
    float INV_HPHTR[4 * 4];
    mat_invert(HPHTR, 4, INV_HPHTR);


    // Kalman_Gain = PHT * INV_HPHTR
    float Kalman_Gain[7 * 4];
    mat_mult(PHT, 7, 4, INV_HPHTR, 4, 4, Kalman_Gain);


    // State Correction_Orientation
    float Measurment_Error[4];
    quaternionDifference(Orientation_Measurment, Predicted_Measurment_Orientation, Measurment_Error);


    float State_Correction[7];
    mat_mult(Kalman_Gain, 7, 4, Measurment_Error, 4, 1, State_Correction);


    for (int i = 0; i < 7; ++i) {
        State_Vector_Orientation[i] += State_Correction[i];
    }

    Fix_Quaternion(State_Vector_Orientation);

    // Covariance_Orientation Correction_Orientation
    float KH[7 * 7];
    mat_mult(Kalman_Gain, 7, 4, H_Orientation, 4, 7, KH);

    float IKH[7 * 7];
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            if (i == j) {
                IKH[i * 7 + j] = 1.0 - KH[i * 7 + j];
            }
            else {
                IKH[i * 7 + j] = -KH[i * 7 + j];
            }
        }
    }

    float new_Covariance_Orientation[7 * 7];
    mat_mult(IKH, 7, 7, Covariance_Orientation, 7, 7, new_Covariance_Orientation);

    for (int i = 0; i < 7 * 7; ++i) {
        Covariance_Orientation[i] = new_Covariance_Orientation[i];
    }
}


// Function to compute the derivative of the state vector for orientation
void Derivative_Orientation(const float* State_Vector_Orientation, const float* IMU_Data, float* State_Vector_Derivative_Orientation) {
    const float* Q = State_Vector_Orientation;
    float bp = State_Vector_Orientation[4];
    float bq = State_Vector_Orientation[5];
    float br = State_Vector_Orientation[6];

    float W_imu_x = IMU_Data[0];
    float W_imu_y = IMU_Data[1];
    float W_imu_z = IMU_Data[2];

    float Quaternian_matrix_thing[4 * 4] = {
        0,               -(W_imu_x - bp),        -(W_imu_y - bq),        -(W_imu_z - br),
        W_imu_x - bp,    0,                      W_imu_z - br,           -(W_imu_y - bq),
        W_imu_y - bq,    -(W_imu_z - br),        0,                      W_imu_x - bp,
        W_imu_z - br,    W_imu_y - bq,           -(W_imu_x - bp),        0
    };

    float Q_dot[4];
    for (int i = 0; i < 4; ++i) {
        Q_dot[i] = 0.0;
        for (int j = 0; j < 4; ++j) {
            Q_dot[i] += 0.5 * Quaternian_matrix_thing[i * 4 + j] * Q[j];
        }
    }

    float bw_dot[3] = { 0.0, 0.0, 0.0 };

    for (int i = 0; i < 4; ++i) {
        State_Vector_Derivative_Orientation[i] = Q_dot[i];
    }
    for (int i = 0; i < 3; ++i) {
        State_Vector_Derivative_Orientation[4 + i] = bw_dot[i];
    }
}


void Find_F_Orientation(const float* State, const float* IMU, float* F) {
    float q0 = State[0];
    float q1 = State[1];
    float q2 = State[2];
    float q3 = State[3];

    float bp = State[4];
    float bq = State[5];
    float br = State[6];

    float W_imu_x = IMU[0];
    float W_imu_y = IMU[1];
    float W_imu_z = IMU[2];

    float F11[4 * 4] = {
        0,               (bp - W_imu_x) / 2, (bq - W_imu_y) / 2, (br - W_imu_z) / 2,
        (-bp + W_imu_x) / 2, 0,               (-br + W_imu_z) / 2, (bq - W_imu_y) / 2,
        (-bq + W_imu_y) / 2, (br - W_imu_z) / 2, 0,               (-bp + W_imu_x) / 2,
        (-br + W_imu_z) / 2, (-bq + W_imu_y) / 2, (bp - W_imu_x) / 2, 0
    };

    float F15[4 * 3] = {
        q1 / 2, q2 / 2, q3 / 2,
        -q0 / 2, q3 / 2, -q2 / 2,
        -q3 / 2, -q0 / 2, q1 / 2,
        q2 / 2, -q1 / 2, -q0 / 2
    };

    // Initialize F with zeros manually
    for (int i = 0; i < 7 * 7; ++i) {
        F[i] = 0.0;
    }

    // Copy F11 into F
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            F[i * 7 + j] = F11[i * 4 + j];
        }
    }

    // Copy F15 into F
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            F[i * 7 + (j + 4)] = F15[i * 3 + j];
        }
    }
}


void Prediction_Orientation(float* State_Vector_Orientation, float* Covariance_Orientation, float Dt, const float* Q_Orientation, const float* IMU_Data, float* New_State_Vector_Derivative) {
    float State_Vector_Derivative_1[7];
    Derivative_Orientation(State_Vector_Orientation, IMU_Data, State_Vector_Derivative_1);

    float Fake_State_Vector[7];
    float sum = 0.0;
    for (int i = 0; i < 7; ++i) {
        Fake_State_Vector[i] = State_Vector_Orientation[i] + State_Vector_Derivative_1[i] * Dt;
        sum += Fake_State_Vector[i] * Fake_State_Vector[i];
    }

    float norm_Fake_State_Vector = std::sqrt(sum);
    for (int i = 0; i < 7; ++i) {
        Fake_State_Vector[i] /= norm_Fake_State_Vector;
    }

    float State_Vector_Derivative_2[7];
    Derivative_Orientation(Fake_State_Vector, IMU_Data, State_Vector_Derivative_2);

    for (int i = 0; i < 7; ++i) {
        New_State_Vector_Derivative[i] = 0.5 * (State_Vector_Derivative_2[i] + State_Vector_Derivative_1[i]);
        State_Vector_Orientation[i] += New_State_Vector_Derivative[i] * Dt;
    }

    Fix_Quaternion(State_Vector_Orientation);

    float F[7 * 7];
    Find_F_Orientation(State_Vector_Orientation, IMU_Data, F);

    float FP[7 * 7];
    mat_mult(F, 7, 7, Covariance_Orientation, 7, 7, FP);

    float F_transpose[7 * 7];
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            F_transpose[j * 7 + i] = F[i * 7 + j];
        }
    }

    float PFT[7 * 7];
    mat_mult(Covariance_Orientation, 7, 7, F_transpose, 7, 7, PFT);

    float Covariance_Derivative[7 * 7];
    for (int i = 0; i < 7 * 7; ++i) {
        Covariance_Derivative[i] = FP[i] + PFT[i] + Q_Orientation[i];
        Covariance_Orientation[i] += Covariance_Derivative[i] * Dt;
    }
}


// Function to compute the Q matrix for the position kalman filter
void Find_Q_Position(const float* IMU_Sigma_Accel, float* Q_Position) {
    float Sig_AX_Squared = IMU_Sigma_Accel[0] * IMU_Sigma_Accel[0];
    float Sig_AY_Squared = IMU_Sigma_Accel[1] * IMU_Sigma_Accel[1];
    float Sig_AZ_Squared = IMU_Sigma_Accel[2] * IMU_Sigma_Accel[2];

    float Q_Accel[3 * 3] = {
        Sig_AX_Squared, 0, 0,
        0, Sig_AY_Squared, 0,
        0, 0, Sig_AZ_Squared
    };

    // Initialize Q_Position with zeros manually
    for (int i = 0; i < 9 * 9; ++i) {
        Q_Position[i] = 0.0;
    }

    // Copy Q_Accel into Q_Position
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Q_Position[(3 + i) * 9 + (3 + j)] = Q_Accel[i * 3 + j];
        }
    }
}


// Function to calculate the H_Position matrix
void Find_H_Position(float* H_Position) {
    // Initialize H_Position with zeros
    for (int i = 0; i < 3 * 9; ++i) {
        H_Position[i] = 0.0;
    }

    // Set the identity matrix part
    for (int i = 0; i < 3; ++i) {
        H_Position[i * 9 + i] = 1.0;
    }
}


// Function to calculate the R matrix for the position measurement
void Find_R_Position(const float* GPS_Sigma_Pos, float* R_Position) {
    float Sig_X = GPS_Sigma_Pos[0];
    float Sig_Y = GPS_Sigma_Pos[1];
    float Sig_Z = GPS_Sigma_Pos[2];

    float R11[3 * 3] = {
        Sig_X * Sig_X, 0, 0,
        0, Sig_Y * Sig_Y, 0,
        0, 0, Sig_Z * Sig_Z
    };

    // Copy R11 into R_Position
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_Position[i * 3 + j] = R11[i * 3 + j];
        }
    }
}


void Correction_Position(
    float* State_Vector_Position,
    float* Covariance_Position,
    float* R_Position,
    float* H_Position,
    float* Real_Measurment_Position,
    float* Predicted_Measurment_Position) {

    // Transpose H_Position
    float H_Transpose[9 * 3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 9; ++j) {
            H_Transpose[j * 3 + i] = H_Position[i * 9 + j];
        }
    }
    // PHT = Covariance_Position * H_Transpose
    float PHT[9 * 3];
    mat_mult(Covariance_Position, 9, 9, H_Transpose, 9, 3, PHT);


    // HP = H_Position * Covariance_Position
    float HP[9 * 3];
    mat_mult(H_Position, 3, 9, Covariance_Position, 9, 9, HP);


    // HPHT = HP * H_Transpose
    float HPHT[3 * 3];
    mat_mult(HP, 3, 9, H_Transpose, 9, 3, HPHT);


    // HPHTR = HPHT + R_Position
    float HPHTR[3 * 3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            HPHTR[i * 3 + j] = HPHT[i * 3 + j] + R_Position[i * 3 + j];
        }
    }


    // INV_HPHTR = inv(HPHTR)
    float INV_HPHTR[3 * 3];
    mat_invert(HPHTR, 3, INV_HPHTR);


    // Kalman_Gain = PHT * INV_HPHTR
    float Kalman_Gain[9 * 3];
    mat_mult(PHT, 9, 3, INV_HPHTR, 3, 3, Kalman_Gain);


    // State Correction_Position
    float Measurment_Error[3];
    for (int i = 0; i < 3; ++i) {
        Measurment_Error[i] = Real_Measurment_Position[i] - Predicted_Measurment_Position[i];
    }


    float State_Correction[9 * 1];
    mat_mult(Kalman_Gain, 9, 3, Measurment_Error, 3, 1, State_Correction);


    for (int i = 0; i < 9; ++i) {
        State_Vector_Position[i] += State_Correction[i];
    }

    // Covariance_Position Correction_Position
    float KH[9 * 9];
    mat_mult(Kalman_Gain, 9, 3, H_Position, 3, 9, KH);

    float IKH[9 * 9];
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            if (i == j) {
                IKH[i * 9 + j] = 1.0 - KH[i * 9 + j];
            }
            else {
                IKH[i * 9 + j] = -KH[i * 9 + j];
            }
        }
    }

    float new_Covariance_Position[9 * 9];
    mat_mult(IKH, 9, 9, Covariance_Position, 9, 9, new_Covariance_Position);

    for (int i = 0; i < 9 * 9; ++i) {
        Covariance_Position[i] = new_Covariance_Position[i];
    }
}


void Find_F_Position(float* Quaternion, float* Fk) {
    float q0 = Quaternion[0];
    float q1 = Quaternion[1];
    float q2 = Quaternion[2];
    float q3 = Quaternion[3];

    float F34[3][3] = {
        {-1 + 2 * (q2 * q2 + q3 * q3), -2 * (q1 * q2 - q0 * q3), -2 * (q1 * q3 + q0 * q2)},
        {-2 * (q0 * q3 + q1 * q2), -1 + 2 * (q1 * q1 + q3 * q3), -2 * (q2 * q3 - q0 * q1)},
        {-2 * (q1 * q3 - q0 * q2), -2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)}
    };

    // Initialize Fk to zero
    for (int i = 0; i < 9 * 9; ++i) {
        Fk[i] = 0.0;
    }

    // Set the eye(3) portion
    for (int i = 0; i < 3; ++i) {
        Fk[i * 9 + i + 3] = 1.0;
    }

    // Set the F34 portion
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Fk[(i + 3) * 9 + j + 6] = F34[i][j];
        }
    }
}


void Derivative_Position(
    float* State_Vector_Position,
    float* IMU_Data,
    float* g,
    float* Quaternion,
    float* State_Vector_Derivative_Position) {

    // Extract bas
    float bas[3];
    for (int i = 0; i < 3; ++i) {
        bas[i] = State_Vector_Position[i + 6];
    }

    // Extract Vel
    float Vel[3];
    for (int i = 0; i < 3; ++i) {
        Vel[i] = State_Vector_Position[i + 3];
    }

    // Extract Imu_Accel_Data
    float Imu_Accel_Data[3];
    for (int i = 0; i < 3; ++i) {
        Imu_Accel_Data[i] = IMU_Data[i + 3];
    }

    // Calculate Pos_Dot
    float Pos_Dot[3];
    for (int i = 0; i < 3; ++i) {
        Pos_Dot[i] = Vel[i];
    }

    // Calculate T (rotation matrix from Quaternion)
    float T[3 * 3];
    Quaternian_Rotation_Matrix(Quaternion, T);

    // Calculate V_Dot = T * (Imu_Accel_Data - bas) + g
    float V_Dot[3] = { 0.0, 0.0, 0.0 };
    float accel_minus_bas[3];
    for (int i = 0; i < 3; ++i) {
        accel_minus_bas[i] = Imu_Accel_Data[i] - bas[i];
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            V_Dot[i] += T[i * 3 + j] * accel_minus_bas[j];
        }
        V_Dot[i] += g[i];
    }

    // ba_dot is zero
    float ba_dot[3] = { 0.0, 0.0, 0.0 };

    // Combine results into State_Vector_Derivative_Position
    for (int i = 0; i < 3; ++i) {
        State_Vector_Derivative_Position[i] = Pos_Dot[i];
        State_Vector_Derivative_Position[i + 3] = V_Dot[i];
        State_Vector_Derivative_Position[i + 6] = ba_dot[i];
    }
}


void Prediction_Position(
    float* State_Vector_Position,
    float* Covariance_Position,
    float Dt,
    float* Q_Position,
    float* IMU_Data,
    float* g,
    float* Quaternion,
    float* New_State_Vector_Derivative) {

    // State Update---------------------------------------------------------

    // Calculate State_Vector_Derivative_1
    float State_Vector_Derivative_1[9];
    Derivative_Position(State_Vector_Position, IMU_Data, g, Quaternion, State_Vector_Derivative_1);

    // Calculate Fake_State_Vector = State_Vector_Position + State_Vector_Derivative_1 * Dt
    float Fake_State_Vector[9];
    for (int i = 0; i < 9; ++i) {
        Fake_State_Vector[i] = State_Vector_Position[i] + State_Vector_Derivative_1[i] * Dt;
    }

    // Calculate State_Vector_Derivative_2
    float State_Vector_Derivative_2[9];
    Derivative_Position(Fake_State_Vector, IMU_Data, g, Quaternion, State_Vector_Derivative_2);

    // Calculate New_State_Vector_Derivative = 1.0 / 2.0 * (State_Vector_Derivative_2 + State_Vector_Derivative_1)
    for (int i = 0; i < 9; ++i) {
        New_State_Vector_Derivative[i] = 0.5 * (State_Vector_Derivative_2[i] + State_Vector_Derivative_1[i]);
    }

    // Update State_Vector_Position = State_Vector_Position + New_State_Vector_Derivative * Dt
    for (int i = 0; i < 9; ++i) {
        State_Vector_Position[i] += New_State_Vector_Derivative[i] * Dt;
    }

    // Covariance_Position Update--------------------------------------------

    // Calculate F
    float F[9 * 9];
    Find_F_Position(Quaternion, F);

    // Calculate FP = F * Covariance_Position
    float FP[9 * 9];
    mat_mult(F, 9, 9, Covariance_Position, 9, 9, FP);

    // Transpose F
    float F_transpose[9 * 9];
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            F_transpose[j * 9 + i] = F[i * 9 + j];
        }
    }

    // Calculate PFT = Covariance_Position * F_transpose
    float PFT[9 * 9];
    mat_mult(Covariance_Position, 9, 9, F_transpose, 9, 9, PFT);

    // Calculate Covariance_Derivative = FP + PFT + Q_Position
    float Covariance_Derivative[9 * 9];
    for (int i = 0; i < 9 * 9; ++i) {
        Covariance_Derivative[i] = FP[i] + PFT[i] + Q_Position[i];
    }

    // Update Covariance_Position = Covariance_Position + Covariance_Derivative * Dt
    for (int i = 0; i < 9 * 9; ++i) {
        Covariance_Position[i] += Covariance_Derivative[i] * Dt;
    }
}


// Function to add noise to orientation data
void Motion_Capture_Orientation_Noise_Adder(float* Orientation_Measurment, const float* Orientation_Sig) {
    // Copy the first 4 elements of State (Quaternion)
    for (int i = 0; i < 4; i++) {
        Orientation_Measurment[i] = Orientation_Measurment[i] + randn() * Orientation_Sig[i];
    }

    // Normalize the quaternion to ensure unit norm
    Fix_Quaternion(Orientation_Measurment);
}



void quaternionDotToAngularRates(const float* q, const float* q_dot, float* omega) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float q0_dot = q_dot[0], q1_dot = q_dot[1], q2_dot = q_dot[2], q3_dot = q_dot[3];

    // Transformation matrix from quaternion derivative to angular velocity
    float W[3][4] = {
        {-q1,  q0,  q3, -q2},
        {-q2, -q3,  q0,  q1},
        {-q3,  q2, -q1,  q0}
    };

    // Compute angular velocity components
    omega[0] = 2.0 * (W[0][0] * q0_dot + W[0][1] * q1_dot + W[0][2] * q2_dot + W[0][3] * q3_dot);
    omega[1] = 2.0 * (W[1][0] * q0_dot + W[1][1] * q1_dot + W[1][2] * q2_dot + W[1][3] * q3_dot);
    omega[2] = 2.0 * (W[2][0] * q0_dot + W[2][1] * q1_dot + W[2][2] * q2_dot + W[2][3] * q3_dot);
}







void updateNavigation (float DT, Sensors sens)
{
	/* truth data */

    
    
	/* navigation filter */
    struct onboardControl_ref* cntrl = &onboardControl;
    struct My_Kalman_ref* EKF = &MyEKF;
    struct My_Kalman_Filter2* Kalman_Filter = EKF->MyEKF_Function_Variables;
    struct My_navout_ref* out = EKF->out;

    Kalman_Filter->EKF_Counter++;


    float g[3] = {0,0,32.2 };
    out->time += DT;

    // //Runs for the first instance of the simulation to initilize things.
    // struct gpsModel_ref* GPS_Info = &gpsModel;
    // struct imuModel_ref* IMU_Info = &imuModel;
    // float* IMU_Sig_XYZ = IMU_Info->s_sigma;
    // float* IMU_Sig_Gyro = IMU_Info->w_sigma;
    // float* IMU_Position = IMU_Info->r;
    // float* Motion_Capture_Sig_Pos = GPS_Info->p_sigma;
    // float* Motion_Capture_Position = GPS_Info->r;
    // float*  Orientation_Motion_Capture_Sigma = Kalman_Filter->Orientation_Motion_Capture_Sigma;

    float IMU_Sig_XYZ[3] = { 0.1, 0.1, 0.1 };
    float IMU_Sig_Gyro[3] = { 0.002, 0.002, 0.002 };
    float IMU_Position[3] = { 0.1360052493, 0, 0.1593772966 }; // WRONG SO VERY WRONG, ALSO NEED TO TAKE INTO ACOUNT CORDINATE SYSTEM OF THE POZYX
    float Motion_Capture_Sig_Pos[3] = { 0.0001, 0.0001, 0.0001 };
    float Motion_Capture_Position[3] = { 0, 0, 0.229659 };
    float Orientation_Motion_Capture_Sigma[4] = { 0.0001, 0.0001, 0.0001, 0.0001 };
    float IMU_Data[6] = {0};

    if (Kalman_Filter->EKF_Counter == 1) {

        Drone.addWaypoint(Waypoint(0, 0, -2, 0.2, 0.2, 2, true, 0.05,0.05));

        //Drone.addWaypoint(Waypoint(0, 5, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(5, 5, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(5, 5 + 33 - 4, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(-5, 5 + 33 - 4, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(-5, 9, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(0, 9, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(0, 5 + 33 - 4 - 4, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(2, 5 + 33 - 4 - 4 - 3, -10, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(2, 5 + 33 - 4 - 4-3, -3, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(2, 5 + 33 - 4 - 4 - 3, -5, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(0, 0, -5, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(0, 0, -0.5, 0.2, 0.2, 2, false, 0.05, 0.05));

        //Drone.addWaypoint(Waypoint(0, 0, 0, 0.2, 0.2, 2, false, 0.05, 0.05));




        
        // 7x7 Covariance_Orientation matrix with diagonal elements 0.1
        
        for (int i = 0; i < 7; i++) {
            Kalman_Filter->Orientation_Covariance[i * 7 + i] = 0.1;
        }

        // 9x9 Covariance_Position matrix with diagonal elements 0.1
        
        for (int i = 0; i < 9; i++) {
            Kalman_Filter->Position_Covariance[i * 9 + i] = 0.1;
        }
        Kalman_Filter->Position_Covariance[6 * 9 + 6] = 0.1; // (7,7)
        Kalman_Filter->Position_Covariance[7 * 9 + 7] = 0.1; // (8,8)
        Kalman_Filter->Position_Covariance[8 * 9 + 8] = 0.1; // (9,9)
        


        

        

        


        

    }

	//---------------------------------------------------------------------------------------------

    if (sens.data_POZYX.POZYX_Update_Counter != Kalman_Filter->IMU_Counter) {
        
        if (out->time <= 4) {
            Kalman_Filter->Position_State[6] = 0;
            Kalman_Filter->Position_State[7] = 0;
            Kalman_Filter->Position_State[8] = 0;

        }
        

        Kalman_Filter->IMU_Counter = sens.data_POZYX.POZYX_Update_Counter;
        
        //Gets IMU Data for current time
        float* Gyroscope_Data = sens.data_POZYX.gyr_rad;
        float* Accelerometer_Data = sens.data_POZYX.acc;


        
        

        // Populate IMU_Data using a for loop
        for (int i = 0; i < 3; i++) {
            IMU_Data[i] = Gyroscope_Data[i];       // First three elements from Gyroscope_Data
            IMU_Data[i + 3] = Accelerometer_Data[i]; // Next three elements from Accelerometer_Data
        }


        
        Correct_IMU_Data(IMU_Data, IMU_Position);
        
        
        
        
        float Q_Orientation[7 * 7] = { 0 };
        float Q_Position[9 * 9] = { 0 };
        Find_Q_Orientation(IMU_Sig_Gyro, Q_Orientation);
        Find_Q_Position(IMU_Sig_XYZ, Q_Position);


        float Predicted_Quaternion[4] = {
        Kalman_Filter->Orientation_State[0],
        Kalman_Filter->Orientation_State[1],
        Kalman_Filter->Orientation_State[2],
        Kalman_Filter->Orientation_State[3]
        };


        Prediction_Orientation(
            Kalman_Filter->Orientation_State, 
            Kalman_Filter->Orientation_Covariance, 
            DT, 
            Q_Orientation, 
            IMU_Data, Kalman_Filter->State_Vector_Derivative_Orientation);

        

        Prediction_Position(
            Kalman_Filter->Position_State,
            Kalman_Filter->Position_Covariance,
            DT,
            Q_Position,
            IMU_Data,
            g,
            Predicted_Quaternion,
            Kalman_Filter->State_Vector_Derivative_Position);


        Fix_Quaternion(Kalman_Filter->Orientation_State);
        if ((sens.data_MoCap.MoCap_Update_Counter != Kalman_Filter->GPS_Counter)) {
            Kalman_Filter->GPS_Counter = sens.data_MoCap.MoCap_Update_Counter;

            float Predicted_Measurment_Orientation[4] = {
            Kalman_Filter->Orientation_State[0],
            Kalman_Filter->Orientation_State[1],
            Kalman_Filter->Orientation_State[2],
            Kalman_Filter->Orientation_State[3]
                };



            

            
            float Real_Measurment_Position[3];
            float Real_Measurment_Orientation[4];

            // Serial.println("Begin");
            // Serial.print(sens.data_MoCap.Pos[0]);
            // Serial.print("\t ");
            // Serial.print(sens.data_MoCap.Pos[1]);
            // Serial.print("\t ");
            // Serial.print(sens.data_MoCap.Pos[2]);
            // Serial.print("\t ");
            // Serial.print(sens.data_MoCap.Quat[0]);
            // Serial.print("\t ");
            // Serial.print(sens.data_MoCap.Quat[1]);
            // Serial.print("\t ");
            // Serial.print(sens.data_MoCap.Quat[2]);
            // Serial.print("\t ");
            // Serial.println(sens.data_MoCap.Quat[3]);
            // Serial.println("End");
            //float* Real_Measurment_Position = (float*)sens.data_MoCap.Pos;
            //float* Real_Measurment_Orientation = (float*)sens.data_MoCap.Quat;
            Real_Measurment_Position[0] = sens.data_MoCap.Pos[0];
            Real_Measurment_Position[1] = sens.data_MoCap.Pos[1];
            Real_Measurment_Position[2] = sens.data_MoCap.Pos[2];

            Real_Measurment_Orientation[0] = sens.data_MoCap.Quat[0];
            Real_Measurment_Orientation[1] = sens.data_MoCap.Quat[1];
            Real_Measurment_Orientation[2] = sens.data_MoCap.Quat[2];
            Real_Measurment_Orientation[3] = sens.data_MoCap.Quat[3];

            // Serial.println("Stuff");
            // for (int i = 0; i < 3; i++) {
            //     Serial.println(Real_Measurment_Position[i], 6); // Print with precision
            // }
            // for (int i = 0; i < 4; i++) {
            //     Serial.println(Real_Measurment_Orientation[i], 6);
            // }
            // Serial.println("Begin");
            // Serial.print(Real_Measurment_Position[0]);
            // Serial.print("\t ");
            // Serial.print(Real_Measurment_Position[1]);
            // Serial.print("\t ");
            // Serial.print(Real_Measurment_Position[2]);
            // Serial.print("\t ");
            // Serial.print(Real_Measurment_Orientation[0]);
            // Serial.print("\t ");
            // Serial.print(Real_Measurment_Orientation[1]);
            // Serial.print("\t ");
            // Serial.print(Real_Measurment_Orientation[2]);
            // Serial.print("\t ");
            // Serial.println(Real_Measurment_Orientation[3]);
            // Serial.println("End");



            Correct_Motion_Capture_Orientation_Data(Real_Measurment_Orientation);

            Correct_Motion_Capture_Position_Data(Real_Measurment_Position,
                Motion_Capture_Position,
                Predicted_Measurment_Orientation);

            
            float Predicted_Measurment_Position[3];
            for (int i = 0; i < 3; i++) {
                Predicted_Measurment_Position[i] = Kalman_Filter->Position_State[i];
            }

            float Rotation_Matrix[9] = { 0 };
            Quaternian_Rotation_Matrix(Predicted_Measurment_Orientation, Rotation_Matrix);



            float R_Orientation[4 * 4] = { 0 };
            float R_Position[3 * 3] = { 0 };
            Find_R_Orientation(Orientation_Motion_Capture_Sigma, R_Orientation);
            Find_R_Position(Motion_Capture_Sig_Pos, R_Position);



            float H_Orientation[4 * 7] = { 0 };
            float H_Position[3 * 9] = { 0 };
            Find_H_Orientation(H_Orientation);
            Find_H_Position(H_Position);



            Correction_Orientation(Kalman_Filter->Orientation_State, Kalman_Filter->Orientation_Covariance, R_Orientation,
                H_Orientation, Real_Measurment_Orientation, Predicted_Measurment_Orientation);


            Correction_Position(
                    Kalman_Filter->Position_State,
                    Kalman_Filter->Position_Covariance,
                    R_Position,
                    H_Position,
                    Real_Measurment_Position,
                    Predicted_Measurment_Position);

            Fix_Quaternion(Kalman_Filter->Orientation_State);


            
        }

    }
    // Serial.println("IMU DATA: ");
    // mat_display(IMU_Data, 1, 6);
    // Serial.println("Orientation State: ");
    // mat_display(Kalman_Filter->Orientation_State, 1, 7);
    // Serial.println("Orientation Covariance: ");
    // mat_display(Kalman_Filter->Orientation_Covariance, 7, 7);
    float q_dot[4] = { 0 };
    float q[4] = { 0 };
    for (int i = 0; i < 4; i++)
    {
        q[i] = Kalman_Filter->Orientation_State[i];
        q_dot[i] = Kalman_Filter->State_Vector_Derivative_Orientation[i];
    }
    quat2euler2(q, &Kalman_Filter->Euler_Angles[0], &Kalman_Filter->Euler_Angles[1], &Kalman_Filter->Euler_Angles[2]);
    quaternionDotToAngularRates(q, q_dot, Kalman_Filter->Angular_Rates_Current_Frame);

    for (int i = 0; i < 3; i++)
    {
        Kalman_Filter->Angular_Rates_Avg[i] = ALPHA * Kalman_Filter->Angular_Rates_Current_Frame[i]
            + (1 - ALPHA) * Kalman_Filter->Angular_Rates_Avg[i];

    }


    for (int i = 0; i < 3; i++)
    {
        out->p_b_e_L[i] = Kalman_Filter->Position_State[i];
        out->v_b_e_L[i] = Kalman_Filter->Position_State[3 + i];
        out->a_b_e_L[i] = Kalman_Filter->State_Vector_Derivative_Position[i+3];
        out->w_b_e_B[i] = Kalman_Filter->Angular_Rates_Avg[i];
    }

    
    for (int i = 0; i < 4; i++)
    {
        out->q[i] = Kalman_Filter->Orientation_State[i];
        
    }
    out->phi = Kalman_Filter->Euler_Angles[0];
    out->theta = Kalman_Filter->Euler_Angles[1];
    out->psi = Kalman_Filter->Euler_Angles[2];

    //quaternionDotToAngularRates(out->q, q_dot, out->w_b_e_B);
    //quat2euler(out->q, &out->phi, &out->theta, &out->psi);
    float Vel = sqrt(out->v_b_e_L[0] * out->v_b_e_L[0] + out->v_b_e_L[1] * out->v_b_e_L[1] + out->v_b_e_L[2] * out->v_b_e_L[2]);
    float* Pose_Des = Drone.update(out->p_b_e_L[0], out->p_b_e_L[1], out->p_b_e_L[2], out->psi, out->w_b_e_B[2], Vel, DT);
    cntrl->posDes[0] = Pose_Des[0];
    cntrl->posDes[1] = Pose_Des[1];
    cntrl->posDes[2] = Pose_Des[2];
    cntrl->psiCmd = Pose_Des[3];
    


}