#ifndef rmax_navigation_h
#define rmax_navigation_h
#include <cmath>
#include <iostream>
#if defined(__cplusplus)
#include <vector>
extern "C"
{
#endif
	void updateNavigation (float DT, Sensors sens);
    void Fix_Quaternion(float* State);
    void Quaternian_Rotation_Matrix(const float* Qs, float* Rotation_Matrix);
    void Correct_IMU_Data(float* IMU_Data, const float* IMU_Position);
    void Correct_Motion_Capture_Orientation_Data(float* Orientation_Data);
    void Correct_Motion_Capture_Position_Data(float* Position_Data,
        const float* Motion_Capture_Sensor_Position,
        const float* Quaternion);
    void Find_Q_Orientation(float* IMU_Sigma_Gyro, float* Q_Orientation);
    void Find_H_Orientation(float* H_Orientation);
    void Find_R_Orientation(const float* Positioning_System_Orientation_Sigmas, float* R_Orientation);
    void quaternionDifference(const float* Q1, const float* Q2, float* delta_q);
    void Correction_Orientation(float* State_Vector_Orientation, float* Covariance_Orientation, float* R_Orientation,
        float* H_Orientation, float* Orientation_Measurment, float* Predicted_Measurment_Orientation);
    void Derivative_Orientation(const float* State_Vector_Orientation, const float* IMU_Data, float* State_Vector_Derivative_Orientation);
    void Find_F_Orientation(const float* State, const float* IMU, float* F);
    void Prediction_Orientation(float* State_Vector_Orientation, float* Covariance_Orientation, float Dt, const float* Q_Orientation, const float* IMU_Data, float* New_State_Vector_Derivative);
    void Find_Q_Position(const float* IMU_Sigma_Accel, float* Q_Position);
    void Find_H_Position(float* H_Position);
    void Find_R_Position(const float* GPS_Sigma_Pos, float* R_Position);
    void Correction_Position(
        float* State_Vector_Position,
        float* Covariance_Position,
        float* R_Position,
        float* H_Position,
        float* Real_Measurment_Position,
        float* Predicted_Measurment_Position);
    void Find_F_Position(float* Quaternion, float* Fk);
    void Derivative_Position(
        float* State_Vector_Position,
        float* IMU_Data,
        float* g,
        float* Quaternion,
        float* State_Vector_Derivative_Position);
    void Prediction_Position(
        float* State_Vector_Position,
        float* Covariance_Position,
        float Dt,
        float* Q_Position,
        float* IMU_Data,
        float* g,
        float* Quaternion,
        float* New_State_Vector_Derivative);
    float randn();

    class Waypoint {
    public:
        float x, y, z; // Coordinates
        float radius; // Distance to consider "at" the waypoint
        float maxVelocity; // Maximum allowed velocity at this waypoint
        float holdTime; // Time to stay at the waypoint
        float elapsedHoldTime; // Time spent at this waypoint
        bool Turn_to;
        float Turn_error;
        float Turn_vel_error;

        Waypoint(float x, float y, float z, float radius, float maxVelocity, float holdTime, bool Turn_to, float Turn_error, float Turn_vel_error)
            : x(x), y(y), z(z), radius(radius), maxVelocity(maxVelocity), holdTime(holdTime), Turn_to(Turn_to), Turn_error(Turn_error), Turn_vel_error(Turn_vel_error), elapsedHoldTime(0) {}

        bool isTurned(float currentAngle,float angle_derivative, float desiredAngle) {
            float angleDifference = fmod(currentAngle - desiredAngle + M_PI, 2 * M_PI) - M_PI;
            std::cout << angle_derivative << std::endl;
            if (abs(angleDifference) <= Turn_error && abs(angle_derivative) <= Turn_vel_error) {
                return 1;
            }
            else {
                return 0;
            }
        }

        bool isAtWaypoint(float droneX, float droneY, float droneZ) {
            float distance = std::sqrt(std::pow(droneX - x, 2) + std::pow(droneY - y, 2) + std::pow(droneZ - z, 2));
            return distance <= radius;
        }

        bool canTransition(float velocity) {
            return elapsedHoldTime >= holdTime && velocity <= maxVelocity;
        }

        void updateHoldTime(float deltaTime) {
            elapsedHoldTime += deltaTime;
        }

        void resetHoldTime() {
            elapsedHoldTime = 0;
        }
    };

    enum State {
        FLYING,
        AT_WAYPOINT,
        HOLD,
        TURNING
    };

    class Finite_State_Machine {
    private:
        State currentState;
        std::vector<Waypoint> waypoints;
        
        int currentWaypointIndex;

    public:
        struct onboardControl_ref* cntrl;
        float psi_des;
        Finite_State_Machine(struct onboardControl_ref* cntrl) :cntrl(cntrl), currentState(FLYING), currentWaypointIndex(0) {}

        void addWaypoint(const Waypoint& waypoint) {
            waypoints.push_back(waypoint);
        }

        float* update(float droneX, float droneY, float droneZ,float psi,float psi_dot, float velocity, float deltaTime) {
            float Pose_Desired[4] = { 0 };
            if (currentWaypointIndex >= waypoints.size()) {
                Waypoint& currentWaypoint = waypoints[currentWaypointIndex - 1];
                Pose_Desired[0] = currentWaypoint.x;
                Pose_Desired[1] = currentWaypoint.y;
                Pose_Desired[2] = currentWaypoint.z;
                Pose_Desired[3] = psi;
                return Pose_Desired;
            }
            else {
                Waypoint& currentWaypoint = waypoints[currentWaypointIndex];
                switch (currentState) {
                case TURNING:
                    if (currentWaypoint.Turn_to) {
                        Waypoint& PreviousWaypoint = waypoints[currentWaypointIndex-1];
                        float Delta_Y = currentWaypoint.y - PreviousWaypoint.y;
                        float Delta_X = currentWaypoint.x - PreviousWaypoint.x;
                        if (Delta_Y == 0 && Delta_X == 0) {
                            std::cout << "DIVIDE BY ZERO No Turning-----------------------------" << std::endl;
                            currentState = FLYING;
                        }else{
                            psi_des = atan2(Delta_Y, Delta_X);
                            std::cout << "Turning" << std::endl;
                            if (currentWaypoint.isTurned(psi,psi_dot, psi_des)) {
                                currentState = FLYING;
                            }
                            else {
                                Pose_Desired[0] = PreviousWaypoint.x;
                                Pose_Desired[1] = PreviousWaypoint.y;
                                Pose_Desired[2] = PreviousWaypoint.z;
                                Pose_Desired[3] = psi_des;
                                return Pose_Desired;
                            }
                        }
                        
                    }
                    else {
                        psi_des = psi;
                        currentState = FLYING;
                    }
                case FLYING:
                    if (currentWaypoint.isAtWaypoint(droneX, droneY, droneZ)) {
                        currentState = AT_WAYPOINT;
                        currentWaypoint.resetHoldTime();
                        std::cout << "Arrived at waypoint " << currentWaypointIndex << std::endl;
                    }
                    break;

                case AT_WAYPOINT:
                    if (currentWaypoint.isAtWaypoint(droneX, droneY, droneZ)) {
                        currentWaypoint.updateHoldTime(deltaTime);
                        std::cout << "Holding at waypoint " << currentWaypointIndex << std::endl;

                        // Check if conditions are met to transition
                        if (currentWaypoint.canTransition(velocity)) {
                            std::cout << "Conditions met, Moving to next waypoint " << currentWaypointIndex << std::endl;
                            currentWaypointIndex++;
                            cntrl->integralPos[0] = 0;
                            cntrl->integralPos[1] = 0;
                            currentState = TURNING;
                        }
                    }
                    else {
                        // If we are no longer at the waypoint, reset hold time
                        currentWaypoint.resetHoldTime();
                        std::cout << "Moved away from waypoint " << currentWaypointIndex << ".\n";
                        currentState = FLYING; // Try to re-approach
                    }
                    break;


                case HOLD:
                    // This state may not be needed with the current logic, but you can use it for additional logic.
                    break;

                }
                Pose_Desired[0] = currentWaypoint.x;
                Pose_Desired[1] = currentWaypoint.y;
                Pose_Desired[2] = currentWaypoint.z;
                Pose_Desired[3] = psi_des;
                return Pose_Desired;
            }

        }
    };
#if defined(__cplusplus)
}
#endif



#endif