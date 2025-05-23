%This program will simulate an orientation and position state estimation
%algorithm. This is done by using 2 kalman filters. The simulation assumes
%that the user is using a motion capture system, where Position and
%orientation updates are avalable

%The first kalman filter will find position and velocity information. It
%will use IMU accelerometer data for prediction, and the position update
%from the motion capture system is used for the correction step. The kalman
%filter will also figure out any IMU biases and correct for them if they exist.

%The second kalman filter is used to figure out orientation. This will use
%the IMU gyroscope data for the prediction step, then it will use the
%orientation information provided by the motion capture system for the
%correction step.

%Combines, these 2 kalman filters can give verry good attitude estimation
%at 100hz, and will be used on the AERSP 403 drone for the FireFlighters
%Team.

%This program uses a lot of equations and ideas from this paper by Dr
%Johnson, however I modified some parts so we can fully utilize the motion
%capture orientation information instead of just GPS.
%Link: https://repository.gatech.edu/server/api/core/bitstreams/c2b0e9f1-8acc-49db-8dc0-da56467eb99b/content

%Programmed by: Mark Yazemboski 





%Clears the workspace, command window, and figures.
clear all;
clc;
clear;

%This section is where you can specify the different time incriments, for
%the simulation its self, and the measurment time step, aswell as the total
%simulation time, ect.
dt  = 0.01;   % process time step
dtm = 0.2;   % measurement time step (multiple of above)
tfinal = 20;  % simulate this many seconds
showRealTimePlot = true;  % turns on and off the visulization
dtAnimate = dt; % update time step of real time plot (in simulated time)


%This section is where all of the sigma values, sensor positions, and
%biases are initilized.

expected_x0_Orientation = [1,0,0,0,0.00,0.00,0.00];    %Initial Orientation State [q0,q1,q2,q3,bwx,bwy,bwz]
expected_x0_Position = [0,0,0,0,0,0,0,0,0];            %Initial Position State [x,y,z,vx,vy,vz,bx,by,bz]

IMU_Pos = [0.1,0.1,0.1]';                  %Position of the IMU
Motion_Capture_Pos = [0.1,0.1,0.1]';       %Position of the Motion capture centerpoint

% IMU_Gyro_Sigma = [0.02,0.02,0.02]';             %Sigma values for the IMU gyroscope
% IMU_Accel_Sigma = [0.05,0.05,0.05]';            %Sigma values for the IMU accelerometer

IMU_Gyro_Sigma = [0.02,0.02,0.02]';             %Sigma values for the IMU gyroscope
IMU_Accel_Sigma = [0.05,0.05,0.05]';            %Sigma values for the IMU accelerometer


IMU_Sig = [IMU_Gyro_Sigma;IMU_Accel_Sigma];     %Combines all of the IMU sigma values into one array

Motion_Capture_Orientation_Sigmas = [0.01,0.01,0.01,0.01]';         %Sigma values for the Orientation information from the motion capture system
Motion_Capture_Pos_Sigma = [0.03,0.03,0.03]';                       %Sigma values for the Position information from the motion capture system

IMU_Accel_Bias = [0.1;0.1;0.1];             %IMU accelerometer Biases
IMU_Gyro_Bias = [0.1;0.1;0.1];              %IMU gyroscope Biases
IMU_Bias = [IMU_Gyro_Bias;IMU_Accel_Bias];  %Combines the IMU Biases into one array





%Calculates the number of iterations, and initilizes a lot of the
%simulation variables.
points  = tfinal/dt  + 1;
mpoints = tfinal/dtm + 1;
multiple        = max(1,floor(dtm/dt));        % make sure this is an integer
multipleAnimate = max(1,floor(dtAnimate/dt));  % make sure this is an integer
Current_Time_For_Measurment = 0;
Current_Time = 0;


%Sets all of the True State, And Estimated state arrays, and measurments
x_Orientation(:,1)    = expected_x0_Orientation;
x_Orientation(5:7) = IMU_Gyro_Bias;
x_dot_Orientation(:,1)    = [0,0,0,0,0,0,0]';
X_Position(:,1)    = expected_x0_Position;
X_Position(7:9) = IMU_Accel_Bias;
X_dot_Position(:,1)    = [0,0,0,0,0,0,0,0,0]';
State_Vector_Orientation = expected_x0_Orientation';
State_Vector_Derivative_Orientation = zeros(7,1);
State_Vector_Position = expected_x0_Position';
State_Vector_Derivative_Position = zeros(9,1);
Orientation_Measurment = [0,0,0,0]';
Real_Measurment_Position = [0,0,0,0,0,0]';



%Sets the Covariance matricies for the position and orientation
Covariance_Orientation = eye(7)*.1;
Covariance_Position = eye(9)*.1;

%Sets the initial IMU acceleration
IMU_Data = [0;0;0;0;0;0];


%Simulates the Drone moving around randomly and the IMU feeling it
Random_Acceleration    = randn(6,points)*0.2;  





%Initilizes Error, and time variables.
Time(1) = 0;
Error_Orientation(1) = 0;
Error_Position(1) = 0;




%Sets gravity 
g = [0,0,999]';




%Starts the figure for showing the simulation
figure(1); 



%Runs the simulation for the set amount of iterations
for k=2:points    
    
    %Gets the current time
    Current_Time=(k-1)*dt;

    
    
    %This section will propigate the true orientation, and position, using
    %an RK4 algorithm, and the random movments, that were created before
    %the simulation loop.
    current_True_omega = quaternionToAngularVelocity(x_Orientation(1:4,k-1), x_dot_Orientation(1:4,k-1));       %Figures out the current true angular velocity
    New_True_Omega = current_True_omega + Random_Acceleration(1:3,k);                                           %Figures out the New true angular velocity by adding the random andgular velocity
    
    [x_Orientation(:, k),x_dot_Orientation(:,k)] = RK4_Orientation(x_Orientation(:, k-1), dt, @RK4_Dynamics_Orientation, x_dot_Orientation(:,k-1),Random_Acceleration(1:3,k) );     %Propigates the orientation
    x_Orientation(:, k) = Fix_Quaternion(x_Orientation(:, k));
    
    [X_Position(:, k),X_dot_Position(:,k)] = RK4_Position(X_Position(:, k-1), dt, @RK4_Dynamics_Position, X_dot_Position(:,k-1), Random_Acceleration(4:6,k),x_Orientation(1:4, k-1));

    %Sets a variable that represents the current predicted quaternion,
    %which will be used in later calculations.
%     Predicted_Quaternion = State_Vector_Orientation(1:4);
    Predicted_Quaternion = x_Orientation(1:4, k-1);


    %Takes in the true state, and will calculate the IMU output, and add
    %noise, and all of the physics that goes on when the IMU is not at the
    %C.G. This basically simulates a real IMU
    IMU_Data = IMU_Noise_Adder(New_True_Omega,X_dot_Position(:,k),IMU_Sig,IMU_Bias,IMU_Pos,g,x_Orientation(1:4, k-1));


    %This will take the Simulated IMU data, and will correct the data for
    %the of C.G location
    IMU_Data = Correct_IMU_Data(IMU_Data,IMU_Pos);
   

   %This is the Prediction_Orientation step. It will look at our IMU data on how we are
   %moving and will propigate our position just using that information
   %instead of relying on a dynamic model of the system.

   %It will take in the previous steps State and convariance, and IMU data,
   %as well as the time step and IMU uncertanty. 

   %There are 2 predictions being made, the orientation, and position. The
   %orientation will be caluclated first, because the position prediction
   %depends on the drones orientation.

   %Finds the Q matrix for the Gyroscope, and accelerometer noise matrix.
   Q_Orientation = Find_Q_Orientation(IMU_Gyro_Sigma);
   Q_Position = Find_Q_Position(IMU_Accel_Sigma);
   

   %Runs the Prediction step and will output the state vector, Covariance_Orientation
   %and Derivative. The first prediction, is for the orientation kalman
   %filter, and the second one is for the position.
   [State_Vector_Orientation,Covariance_Orientation,State_Vector_Derivative_Orientation] = Prediction_Orientation(State_Vector_Orientation,Covariance_Orientation,dt,Q_Orientation,IMU_Data);
   [State_Vector_Position,Covariance_Position,State_Vector_Derivative_Position] = Prediction_Position(State_Vector_Position,Covariance_Position,dt,Q_Position,IMU_Data,g,Predicted_Quaternion);

   %If during the prediction, the Quaternion deviated from a norm of 1, or
   %turned negitive this will renormilize it so it wont break anything down
   %the line.
   State_Vector_Orientation = Fix_Quaternion(State_Vector_Orientation);
    

   


   
%    Prints prediction data for every prediction step.
%    disp("Prediction_Orientation------------------------------------------")
%    disp("State")
%    disp(State_Vector_Orientation)
%    disp("Covariance_Orientation")
%    disp(Covariance_Orientation)
%    disp("True")
%    disp(x_Orientation(:,k))
% 
%    disp("Prediction_Position------------------------------------------")
%    disp("State")
%    disp(State_Vector_Position)
%    disp("Covariance_Position")
%    disp(Covariance_Position)

   
   
   %This will trigger when the sensor is ready to give a measurment. This
   %is where the correction step will take place.
   if mod(k-1,multiple) == 0

       %Keeps track of what measurment we are on and the measurment time
       km = (k-1)/multiple;
       Current_Time_For_Measurment = (km-1)*dtm;


       %Sets the newest predicted quaternion, this will be used in the correction
       %step
       Newest_Predicted_Quaternion = State_Vector_Orientation(1:4);


       

       %This will simulate the motion capture system. It will take in the
       %currect true state, both orientation, and position, and will add
       %noise to the data, and add in any effects from the sensor not being
       %at the C.G.
       Orientation_Measurment = Motion_Capture_Orientation_Noise_Adder(x_Orientation(:,k),Motion_Capture_Orientation_Sigmas);
       Real_Measurment_Position = Motion_Capture_Position_Noise_Adder(X_Position(:,k),Motion_Capture_Pos_Sigma,Motion_Capture_Pos,x_Orientation(1:4,k));

       %This will take in the simulated measurment values, and correct them
       %for any of C.G effects
       Orientation_Measurment = Correct_Motion_Capture_Orientation_Data(Orientation_Measurment);
       Real_Measurment_Position = Correct_Motion_Capture_Position_Data(Real_Measurment_Position,Motion_Capture_Pos,Newest_Predicted_Quaternion);

       %This will look at the current predicted state of the drone, and
       %will see what a predicted measurment should look like.
       Predicted_Measurment_Orientation = State_Vector_Orientation(1:4);
       Predicted_Measurment_Position = State_Vector_Position(1:3);
        
       %Calculates the rotation matrix of the drone currently.
       Rotation_Matrix = Quaternian_Rotation_Matrix(Newest_Predicted_Quaternion);
        
       %Finds the R matrix for the Orientation, and Position, The R matrix
       %is the expected covariance of the measurment.
       R_Orientation = Find_R_Orientation(Motion_Capture_Orientation_Sigmas);
       R_Position = Find_R_Position(Motion_Capture_Pos_Sigma);

       %This will find the H matrix for the Orientation, and Position
       %measurment. These matricies, basically explain which states are
       %being measured.
       H_Orientation = Find_H_Orientation();
       H_Position = Find_H_GPS_Measurment();

       %Runs the correction step. This will take in the measurment data for
       %the orientation, and position, and will use the covariance matrix,
       %and correct the current state,
       [State_Vector_Orientation,Covariance_Orientation] = Correction_Orientation(State_Vector_Orientation,Covariance_Orientation,R_Orientation,H_Orientation,Orientation_Measurment,Predicted_Measurment_Orientation);
       [State_Vector_Position,Covariance_Position] = Correction_Position(State_Vector_Position,Covariance_Position,R_Position,H_Position,Real_Measurment_Position,Predicted_Measurment_Position);

       %If during the correction, the Quaternion deviated from a norm of 1, or
       %turned negitive this will renormilize it so it wont break anything down
       %the line.
       State_Vector_Orientation = Fix_Quaternion(State_Vector_Orientation);

       % Prints correction data for every correction step.
%        disp("Correction_Orientation------------------------------------------")
%        disp("State")
%        disp(State_Vector_Orientation)
%        disp("Covariance_Orientation")
%        disp(Covariance_Orientation)


%        disp("Correction_Position------------------------------------------")
%        disp("State")
%        disp(State_Vector_Position)
%        disp("Covariance_Position")
%        disp(Covariance_Position)
    
   end



   %Calculates the error between the estimated orientation, and the true
   %orientation, aswell as the error between the estimated position, and
   %the true position
   Error_Position(k) = norm(State_Vector_Position(1:3) - X_Position(1:3,k));
   Error_Orientation(k) = norm(quaternionDifference(State_Vector_Orientation(1:4), x_Orientation(1:4,k)));

   %This will propigate the simulation time
   Time(k) = Time(k-1) + dt;
   

   %Displays the corrent simulation time.
   fprintf("Current Time : %g\n",Current_Time) % display current time

    %This will plot the simulated drone, the true, and the estimated. in
    %both position, and orientation.
   if showRealTimePlot
       if mod(k-1,multipleAnimate) == 0
          poseplot
            xlabel("North-x (m)")
            ylabel("East-y (m)")
            zlabel("Down-z (m)");
            poseplot(quaternion(State_Vector_Orientation(1:4)'),State_Vector_Position(1:3))
            hold on
            poseplot(quaternion(x_Orientation(1:4,k)'),X_Position(1:3,k))
          drawnow
          clf
       end
   end
   
end


%Plots the orientation error vs time
figure(2)
plot(Time,Error_Orientation)
title("Error in orientation VS time")
xlabel('Time');
ylabel('Error_Orientation');

%Plots the position error vs time
figure(3)
plot(Time,Error_Position)
title("Error in position VS time")
xlabel('Time');
ylabel('Error_Position');



%Prints the final estimation, and true state.
disp(State_Vector_Position)
disp(State_Vector_Orientation)
disp(x_Orientation(:,k))


%Prints the average error 
disp("Average Orientation Error")
disp(mean(Error_Orientation))
disp("Average Position Error")
disp(mean(Error_Position))

%This will take in the Raw IMU data, and will correct it for errors caused
%by the IMU being off the C.G
function IMU_Data = Correct_IMU_Data(IMU_Data,IMU_Position)
    %Gyro_Correction (Nothing)
    Angular_Vel = IMU_Data(1:3);
    

    %Accel Correction
    Accel = IMU_Data(4:6);
    Euler_Force = [0;0;0];
    Centrifugal_Force = cross(Angular_Vel,cross(Angular_Vel,IMU_Position));
    Accel_CG = Accel - Euler_Force - Centrifugal_Force;
    
    %Combines the corrected values, into one IMU Array
    IMU_Data = [Angular_Vel;Accel_CG];
end


%This will take in the Raw motioncapture orientation data and will correct it if its not placed
%at the CG
function Orientation_Data = Correct_Motion_Capture_Orientation_Data(Orientation_Data)
    %Really, there is no correction that is needed for orientation
    Orientation_Data = Fix_Quaternion(Orientation_Data);
end


%This will take in the Raw GPS data and will correct it if its not placed
%at the CG.
function Position_Data = Correct_Motion_Capture_Position_Data(Position_Data,Motion_Capture_Sensor_Position,Quaternion)
    T = Quaternian_Rotation_Matrix(Quaternion);

    Position = Position_Data(1:3);
    Corrected_Position = Position - T * Motion_Capture_Sensor_Position;

    Position_Data = Corrected_Position;
end


%This will add noise into the simulated measurment, it will also add in the
%effect of the GPS being located off the CG.
function Noisy_GPS_Data  = Motion_Capture_Position_Noise_Adder(State,GPS_Sig,GPS_Position,Quaternion)
    T = Quaternian_Rotation_Matrix(Quaternion);
    Random_Numbers    = randn(3); 
    Noisy_GPS_Data = State(1:3);

    %Adds in the random noise
    for i=1:3
        Noisy_GPS_Data(i) = Noisy_GPS_Data(i) + Random_Numbers(i)*GPS_Sig(i);
    end
    
    %Moves the Motion capture Position real signal to the simulated
    %position of the Motion capture system
    Noisy_GPS_Data = Noisy_GPS_Data(1:3) + T * GPS_Position;
end


%This function will ensure that the quaternion has a magnitude of 1, and is
%always positive
function State = Fix_Quaternion(State)
    % Extract quaternion components
    q0 = State(1);
    q1 = State(2);
    q2 = State(3);
    q3 = State(4);

    % Calculate the norm of the quaternion
    norm_q = sqrt(q0^2 + q1^2 + q2^2 + q3^2);

    % Avoid division by zero in case of a very small norm
    if norm_q > 1e-8
        % Renormalize each component
        State(1) = q0 / norm_q;
        State(2) = q1 / norm_q;
        State(3) = q2 / norm_q;
        State(4) = q3 / norm_q;
    else
        % Handle edge case (set to a default unit quaternion)
        State(1) = 1.0;
        State(2) = 0.0;
        State(3) = 0.0;
        State(4) = 0.0;
    end

    % Flip the sign of the quaternion if the scalar part is negative
    if q0 < 0
        State(1) = -State(1);
        State(2) = -State(2);
        State(3) = -State(3);
        State(4) = -State(4);
    end
end


%This will add noise into the simulated measurment, it will also add in the
%effect of the GPS being located off the CG
function Noisy_Orientation_Data  = Motion_Capture_Orientation_Noise_Adder(State,Orientation_Sig)
    Random_Numbers    = randn(4); 
    Noisy_Orientation_Data = State(1:4);

    %Adds the random noise
    for i=1:4
        Noisy_Orientation_Data(i) = Noisy_Orientation_Data(i) + Random_Numbers(i)*Orientation_Sig(i);
    end
    
    %Makes sure the norm is 1
    Noisy_Orientation_Data = Noisy_Orientation_Data / norm(Noisy_Orientation_Data);

end


%This will add noise into the simulated IMU, it will also add in the
%effect of the IMU being located off the C.G.
function Noisy_IMU_Data  = IMU_Noise_Adder(angular_velocity,State_dot,IMU_Sig,IMU_Bias,IMU_Position,g,Quaternion)
    T = Quaternian_Rotation_Matrix(Quaternion);
    Random_Numbers    = randn(6); 
    Noisy_IMU_Data = [angular_velocity;State_dot(4:6)];


    %Adds random news, and IMU biases
    for i=1:6
        Noisy_IMU_Data(i) = Noisy_IMU_Data(i) + Random_Numbers(i)*IMU_Sig(i)  + IMU_Bias(i)  ;
    end
    %Adds the effect of the IMU off the C.G
    Euler_Force = [0,0,0]';
    Centrifugal_Force = cross(angular_velocity,cross(angular_velocity,IMU_Position));
    Noisy_IMU_Data(4:6) = Noisy_IMU_Data(4:6) + Euler_Force + Centrifugal_Force - T'*g;
end


%Finds the process error matrix Q
function Q_Orientation = Find_Q_Orientation(IMU_Sigma_Gyro)

    average_Sigma_Gyro = (IMU_Sigma_Gyro(1)+IMU_Sigma_Gyro(2)+IMU_Sigma_Gyro(3))/3;
    Q_Quaternion = eye(4)*average_Sigma_Gyro^2/4;

    Q_Orientation = [Q_Quaternion,zeros(4,3);
        zeros(3,4),zeros(3,3)];
end


%Calculates the H_Orientation matrix
function H_Orientation = Find_H_Orientation()
    H_Orientation = [eye(4),zeros(4,3)];
end


%Calculates the R_Orientation matrix
function R_Orientation = Find_R_Orientation(Positioning_System_Orientation_Sigmas)
    Sig_Q0 = Positioning_System_Orientation_Sigmas(1);
    Sig_Q1 = Positioning_System_Orientation_Sigmas(2);
    Sig_Q2 = Positioning_System_Orientation_Sigmas(3);
    Sig_Q3 = Positioning_System_Orientation_Sigmas(4);

    R_Orientation = [Sig_Q0^2,0,0,0;
        0,Sig_Q1^2,0,0;
        0,0,Sig_Q2^2,0;
        0,0,0,Sig_Q3^2];

end


%Impliments the Correction_Orientation step of the kalman filter
function [State_Vector_Orientation,Covariance_Orientation] = Correction_Orientation(State_Vector_Orientation,Covariance_Orientation,R_Orientation,H_Orientation,Orientation_Measurment,Predicted_Measurment_Orientation)
    

    %Kalman Gain Calculation---------------------------------------
    
    H_Transpose = H_Orientation';

    PHT = Covariance_Orientation * H_Transpose;

    HP = H_Orientation * Covariance_Orientation;

    HPHT = HP * H_Transpose;

    HPHTR = HPHT+R_Orientation;

    INV_HPHTR = inv(HPHTR);

    Kalman_Gain = PHT * INV_HPHTR;

    %State Correction_Orientation----------------------------------------------
    Measurment_Error = quaternionDifference(Orientation_Measurment, Predicted_Measurment_Orientation);

    State_Correction = Kalman_Gain * Measurment_Error;
    
    State_Vector_Orientation = State_Vector_Orientation + State_Correction;

    State_Vector_Orientation = Fix_Quaternion(State_Vector_Orientation);

    
    %Covariance_Orientation Correction_Orientation ------------------------------------------

    KH = Kalman_Gain * H_Orientation;

    IKH = eye(7) - KH;

    Covariance_Orientation = IKH * Covariance_Orientation;

end


%Impliments the Prediction_Orientation step of the kalman filter
function [State_Vector_Orientation,Covariance_Orientation,New_State_Vector_Derivative] = Prediction_Orientation(State_Vector_Orientation,Covariance_Orientation,Dt,Q_Orientation,IMU_Data)
    
    %State Update---------------------------------------------------------

    State_Vector_Derivative_1 = Derivative_Orientation(State_Vector_Orientation,IMU_Data);

    %second order numeric integration
    Fake_State_Vector = State_Vector_Orientation + State_Vector_Derivative_1*Dt;
    Fake_State_Vector = Fake_State_Vector/norm(Fake_State_Vector);
    State_Vector_Derivative_2 = Derivative_Orientation(Fake_State_Vector,IMU_Data);
    New_State_Vector_Derivative = 1.0/2.0 * (State_Vector_Derivative_2+State_Vector_Derivative_1);
    State_Vector_Orientation = State_Vector_Orientation + New_State_Vector_Derivative*Dt;

    %Fixes the quaternion, since the numeric integration will make it not
    %have a norm of 1
    State_Vector_Orientation = Fix_Quaternion(State_Vector_Orientation);
    

    %Covariance_Orientation Update----------------------------------------
    
    F = Find_F_Orientation(State_Vector_Orientation,IMU_Data);

    FP = F*Covariance_Orientation;

    F_transpose = F';

    PFT = Covariance_Orientation*F_transpose;

    Covariance_Derivative = FP + PFT + Q_Orientation;

    Covariance_Orientation = Covariance_Orientation + Covariance_Derivative*Dt;

end


%This will find the F matrix for the Orientation kalman filter
function F = Find_F_Orientation(State,IMU)
    q0 = State(1);
    q1 = State(2);
    q2 = State(3);
    q3 = State(4);


    bp = State(5);
    bq = State(6);
    br = State(7);

    W_imu_x = IMU(1);
    W_imu_y = IMU(2);
    W_imu_z = IMU(3);


    F11 = 1.0/2.0*[0                ,    bp-W_imu_x     ,   bq - W_imu_y     ,   br - W_imu_z;
               -bp + W_imu_x    ,    0              ,   -br + W_imu_z    ,   bq - W_imu_y;
               -bq + W_imu_y    ,    br - W_imu_z   ,    0               ,   -bp + W_imu_x;
               -br + W_imu_z    ,    -bq+W_imu_y    ,    bp-W_imu_x      ,   0];


    F15 = 1.0/2.0*[  q1 , q2 , q3;
                    -q0 , q3 ,-q2;
                    -q3 ,-q0 , q1;
                     q2 ,-q1 ,-q0];
    
    
    F = [F11,  F15;
          
          zeros(3,4),  zeros(3,3)];

end


%Finds the Derivative of the state vector for orientation
function State_Vector_Derivative_Orientation = Derivative_Orientation(State_Vector_Orientation,IMU_Data)

    Q = State_Vector_Orientation(1:4);
    
    bp = State_Vector_Orientation(5);
    bq = State_Vector_Orientation(6);
    br = State_Vector_Orientation(7);


    W_imu_x = IMU_Data(1);
    W_imu_y = IMU_Data(2);
    W_imu_z = IMU_Data(3);


    Quaternian_matrix_thing = [0,               -(W_imu_x - bp),        -(W_imu_y - bq),        -(W_imu_z - br);
                               W_imu_x-bp,      0,                      W_imu_z - br,           -(W_imu_y - bq);
                               W_imu_y - bq,    -(W_imu_z - br),        0,                      W_imu_x-bp;
                               W_imu_z - br,    W_imu_y - bq,           -(W_imu_x - bp),        0];

    Q_dot = 1.0/2.0 * Quaternian_matrix_thing * Q;


    bw_dot = [0;0;0];

    State_Vector_Derivative_Orientation = [Q_dot;bw_dot];

end


%Calculates the rotation matrix of the drone using the quaternions
function Rotation_Matrix = Quaternian_Rotation_Matrix(Qs)
    Q0 = Qs(1);
    Q1 = Qs(2);
    Q2 = Qs(3);
    Q3 = Qs(4);

    Rotation_Matrix = [Q0^2+Q1^2-Q2^2-Q3^2,         2*(Q1*Q2+Q0*Q3),            2*(Q1*Q3 - Q0*Q2);
                        2*(Q1*Q2 - Q0*Q3),          Q0^2-Q1^2+Q2^2-Q3^2         2*(Q2*Q3+Q0*Q1);
                        2*(Q1*Q3+Q0*Q2),            2*(Q2*Q3-Q0*Q1)             Q0^2-Q1^2-Q2^2+Q3^2];

    Rotation_Matrix = Rotation_Matrix';
end


%Finds q_dot given an angular velocity, and quaternion
function q_dot = quaternion_derivative(quaternion, angular_velocity)


    % Extract components of the angular velocity
    p = angular_velocity(1); 
    q = angular_velocity(2); 
    r = angular_velocity(3); 

    % Construct the quaternion-to-angular velocity matrix
    Omega = 0.5 * [
         0,  -p,  -q,  -r;
         p,   0,   r,  -q;
         q,  -r,   0,   p;
         r,   q,  -p,   0
    ];

    % Compute the quaternion derivative
    q_dot = Omega * quaternion;
end


%Obtains Angular velocity, given q_dot, and a quaternion
function omega = quaternionToAngularVelocity(q, q_dot)
    
    
    % Conjugate of quaternion q
    q_conjugate = [q(1); -q(2:4)];
    
    % Quaternion multiplication q_dot * q_conjugate
    q_dot_q_conj = [
        q_dot(1)*q_conjugate(1) - q_dot(2:4)' * q_conjugate(2:4);
        q_dot(1)*q_conjugate(2:4) + q_conjugate(1)*q_dot(2:4) + ...
        cross(q_dot(2:4), q_conjugate(2:4))
    ];
    
    % Angular velocity
    omega = 2 * q_dot_q_conj(2:4);
end


%Since the quaternions can be positive, and negitive, and still describe
%the same rotation, this function will give a reliable way to find the
%difference between them.
function delta_q = quaternionDifference(Q1, Q2)

    % Align signs of quaternions
    if dot(Q1, Q2) < 0
        Q1 = -Q1;
    end

    % Compute element-wise difference
    delta_q = Q1 - Q2;
end


%This function is the dynamics of the orientation, which will be used by
%the rK4 algorithm, to very accuratly propigate the True drone.
function x_dot_Orientation = RK4_Dynamics_Orientation(state,state_dot, random_acceleration)
    % Extract quaternion and angular velocity
    q = state(1:4)/norm(state(1:4)); % Quaternion
    q_dot = state_dot(1:4);

    % Compute angular velocity
    omega = quaternionToAngularVelocity(q, q_dot);

    % Add random acceleration to angular velocity
    omega_new = omega + random_acceleration;

    % Quaternion derivative
    q_dot = quaternion_derivative(q, omega_new);

    % State derivative
    x_dot_Orientation = [q_dot; zeros(3,1)];
end


%RK4 algorithm, which will accuratly propigate the true drone's orientation
function [State, State_Dot] = RK4_Orientation(state, dt, dynamics, varargin)
  
    % RK4_Orientation intermediate steps
    k1 = dynamics(state, varargin{:});
    k2 = dynamics(state + 0.5 * dt * k1, varargin{:});
    k3 = dynamics(state + 0.5 * dt * k2, varargin{:});
    k4 = dynamics(state + dt * k3, varargin{:});

    % Combine steps to compute the next state
    State_Dot = (k1 + 2 * k2 + 2 * k3 + k4)/6;
    State = state + State_Dot*dt;

    %Makes sure that if the quaternion is negitive, it will also flip the
    %Q_dot.
    if State(1) <0
        State_Dot = -State_Dot;
    end

    %Makes sure the quaternion is normalized and positive
    State = Fix_Quaternion(State);
end


%Q matrix for the position kalman filter. The Q matrix is the proccess
%noise of the IMU caused by the accelerometer in this case.
function Q_Position = Find_Q_Position(IMU_Sigma_Accel)


    Sig_AX_Squared = IMU_Sigma_Accel(1)^2;
    Sig_AY_Squared = IMU_Sigma_Accel(2)^2;
    Sig_AZ_Squared = IMU_Sigma_Accel(3)^2;

    Q_Accel = [  Sig_AX_Squared,   0,                0;
                 0,                Sig_AY_Squared,   0;
                 0,                0,                Sig_AZ_Squared];

    Q_Position = [
         zeros(3,3),    zeros(3,3),     zeros(3,3);
         zeros(3,3),    Q_Accel,        zeros(3,3);
         zeros(3,3),    zeros(3,3),     zeros(3,3)];

end


%Calculates the H_Position matrix
function H_Position = Find_H_GPS_Measurment()
    H_Position = [ eye(3), zeros(3,6)];
end

%Calculates the R matrix for the position measurment. This matrix
%represents the measurment uncertanty. Since the only measurment is
%position in this case, the R matrix is very simple.
function R_Position = Find_R_Position(GPS_Sigma_Pos)

    Sig_X = GPS_Sigma_Pos(1);
    Sig_Y = GPS_Sigma_Pos(2);
    Sig_Z = GPS_Sigma_Pos(3);

    R11 = [Sig_X^2,   0,         0;
           0,         Sig_Y^2,   0;
           0,         0,         Sig_Z^2];

    R_Position = R11;
end


%Impliments the Correction step for the position kalman filter. This will
%take in the current covariance matrix, and the measurment data, and will
%give back a corrected state that takes the measurment in mind.
function [State_Vector_Position,Covariance_Position] = Correction_Position(State_Vector_Position,Covariance_Position,R_Position,H_Position,Real_Measurment_Position,Predicted_Measurment_Position)
    

    %Kalman Gain Calculation---------------------------------------
    
    H_Transpose = H_Position';

    PHT = Covariance_Position * H_Transpose;

    HP = H_Position * Covariance_Position;

    HPHT = HP * H_Transpose;

    HPHTR = HPHT+R_Position;

    INV_HPHTR = inv(HPHTR);

    Kalman_Gain = PHT * INV_HPHTR;


    %State Correction_Position----------------------------------------------

    Measurment_Error = Real_Measurment_Position - Predicted_Measurment_Position;

    State_Correction = Kalman_Gain * Measurment_Error;
    
    State_Vector_Position = State_Vector_Position + State_Correction;

    
    %Covariance_Position Correction_Position ------------------------------------------

    KH = Kalman_Gain * H_Position;

    IKH = eye(9) - KH;

    Covariance_Position = IKH * Covariance_Position;

end


%Impliments the Prediction step for the Position kalman filter. This will
%take in the IMU data, and the current state, and will predict how the
%drone should be propiated to the next step.
function [State_Vector_Position,Covariance_Position,New_State_Vector_Derivative] = Prediction_Position(State_Vector_Position,Covariance_Position,Dt,Q_Position,IMU_Data,g,Quaternion)
    
    %State Update---------------------------------------------------------


    State_Vector_Derivative_1 = Derivative_Position(State_Vector_Position,IMU_Data,g,Quaternion);

    Fake_State_Vector = State_Vector_Position + State_Vector_Derivative_1*Dt;

    State_Vector_Derivative_2 = Derivative_Position(Fake_State_Vector,IMU_Data,g,Quaternion);

    New_State_Vector_Derivative = 1.0/2.0 * (State_Vector_Derivative_2+State_Vector_Derivative_1);

    State_Vector_Position = State_Vector_Position + New_State_Vector_Derivative*Dt;

    
    %Covariance_Position Update---------
    
    F = Find_F_Position(Quaternion);

    FP = F*Covariance_Position;

    F_transpose = F';

    PFT = Covariance_Position*F_transpose;

    Covariance_Derivative = FP + PFT + Q_Position;

    Covariance_Position = Covariance_Position + Covariance_Derivative*Dt;

end


%This will find the F matrix for the position kalman filter.
function Fk = Find_F_Position(Quaternion)
    q0 = Quaternion(1);
    q1 = Quaternion(2);
    q2 = Quaternion(3);
    q3 = Quaternion(4);


    F34 = [-1 + 2*(q2^2 + q3^2)   ,    -2*(q1*q2 - q0*q3)     ,    -2*(q1*q3 + q0*q2);
            -2*(q0*q3 + q1*q2)    ,    -1 + 2*(q1^2 + q3^2)   ,    -2*(q2*q3 - q0*q1);
            -2*(q1*q3 - q0*q2)    ,    -2*(q0*q1 + q2*q3)     ,    -1 + 2*(q1^2 + q2^2)];
    
    
    Fk = [
            zeros(3,3),  eye(3),      zeros(3,3),  ;
            zeros(3,3),  zeros(3,3)   F34,         ;
            zeros(3,3),  zeros(3,3),  zeros(3,3),  ];

end


%Finds the Derivative for the position kalman filter. This will be used to
%propigate the drone in the prediction step.
function State_Vector_Derivative_Position = Derivative_Position(State_Vector_Position,IMU_Data,g,Quaternion)

    bas = State_Vector_Position(7:9);

    Vel = State_Vector_Position(4:6);

    Imu_Accel_Data = IMU_Data(4:6);

    Pos_Dot = Vel;
    
    T = Quaternian_Rotation_Matrix(Quaternion);
    V_Dot = T*(Imu_Accel_Data - bas) + g;

    ba_dot = [0;0;0];

    State_Vector_Derivative_Position = [Pos_Dot;V_Dot;ba_dot];

end


%This is the dynamics of the drone in terms of position and velocity
%information. This function is used by the RK4 algorithm to accuratly
%propigate the True drones position state.
function x_dot_Orientation = RK4_Dynamics_Position(state,state_dot, random_acceleration,Quaternion)
    T = Quaternian_Rotation_Matrix(Quaternion);

    V_Dot = T*(state_dot(4:6) + random_acceleration);

    x_dot_Orientation = [state(4:6);V_Dot;zeros(3,1)];
    
end


%This will propigate the true drones position using an RK4 algorithm. This
%will very accuratly predict, where the drone in truth should be.
function [State, State_Dot] = RK4_Position(state, dt, dynamics, varargin)

    % RK4_Orientation intermediate steps
    k1 = dynamics(state, varargin{:});
    k2 = dynamics(state + 0.5 * dt * k1, varargin{:});
    k3 = dynamics(state + 0.5 * dt * k2, varargin{:});
    k4 = dynamics(state + dt * k3, varargin{:});

    % Combine steps to compute the next state
    State_Dot = (k1 + 2 * k2 + 2 * k3 + k4)/6;
    State = state + State_Dot*dt;
end