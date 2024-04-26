l1 = 0.5;
l2 =  0.5;

[q,q1] = IK(0.1, 0,l1,l2);
[qf1,qf2] = IK(0.9,0,l1,l2);
q0 = [q,q1,0];
qf = [qf1,qf2,0];
t0 = 0
tf = 4
t = linspace(t0,tf,40);

%% 
des_qdd = 0.8727 * ones(1,3);
q_full = [];
qd_full = [];
qdd_full = [];
% qd was set to be 0
for i = 1:3
    qdd = des_qdd(i);
    tb = 0.5*(tf-t0) - 0.5*sqrt(qdd^2*(tf-t0)^2 - 4*abs(qdd)*abs(qf(i)-q0(i)))/abs(qdd);
    if q0(i) < qf(i)
        ab0 = [1 t0 t0^2;0 1 2 * t0;0 0 2] \ [q0(i) 0 qdd]'; % Coefficient for first blend
        abf = [1 tf tf^2;0 1 2 * tf;0 0 2] \ [qf(i) 0 -qdd]'; % Coefficient for second blend
    else
        ab0 = [1 t0 t0^2;0 1 2 * t0;0 0 2] \ [q0(i) 0 -qdd]'; % Coefficient for first blend
        abf = [1 tf tf^2;0 1 2 * tf;0 0 2] \ [qf(i) 0 qdd]'; % Coefficient for second blend
    end
    qb1 = ab0(1) + ab0(2) * tb + ab0(3) * tb^2;
    qb2 = abf(1) + abf(2) * (tf - tb) + abf(3) * (tf - tb)^2;
    a = [1 tb;1 (tf - tb)] \ [qb1;qb2]; % Coefficient for linear region
    % first parabolic region
    t11 = t((t0<=t) & (t<=t0+tb));
    q = ab0(1) + ab0(2)*t11 + ab0(3)*t11.^2;
    qd  = ab0(2) + 2*ab0(3)*t11;
    qdd = 2*ab0(3)*ones(size(t11));
    % Linear region
    t22 = t((t0+tb<t) & (t<tf-tb)); % linear region
    q = [q, a(1) + a(2)*t22];
    qd = [qd, a(2).*ones(size(t22))];
    qdd = [qdd, zeros(size(t22))];
    % second parabolic region
    t33 = t((tf-tb<=t) & (t<=tf)); 
    q   = [q, abf(1) + abf(2)*t33 + abf(3)*t33.^2];
    qd  = [qd, abf(2) + 2*abf(3)*t33];
    qdd = [qdd, 2*abf(3)*ones(size(t33))];
    q_full = [q_full q'];
    qd_full = [qd_full qd'];
    qdd_full = [qdd_full qdd'];
end


% Determine the number of joints from your data
num_joints = size(q_full, 2); % This should be 3 based on your setup

% Time vector adjusted for plotting based on your simulation
time_vector = linspace(t0, tf, 40);

% Plotting trajectories for all joints
for joint_num = 1:num_joints
    figure; % New figure for each joint
    % Position (q)
    subplot(3, 1, 1);
    plot(time_vector, q_full(:, joint_num), 'LineWidth', 2);
    title(['Joint ', num2str(joint_num), ' Position']);
    xlabel('Time (s)');
    ylabel('Position (rad)'); % Change units if necessary
    grid on; % Add grid for better readability

    % Velocity (qd)
    subplot(3, 1, 2);
    plot(time_vector, qd_full(:, joint_num), 'LineWidth', 2);
    title(['Joint ', num2str(joint_num), ' Velocity']);
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)'); % Change units if necessary
    grid on;

    % Acceleration (qdd)
    subplot(3, 1, 3);
    plot(time_vector, qdd_full(:, joint_num), 'LineWidth', 2);
    title(['Joint ', num2str(joint_num), ' Acceleration']);
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)'); % Change units if necessary
    grid on;
end

%% Compute the joint torques as a function of time 
rho = 2710;
m1 = l1*pi*(0.05^2-0.0495^2)*rho;
m2 = m1;
g = 9.8;

for k = 1:1:40

    t1 = q_full(k,1);
    t2 = q_full(k,2);
    dt1 = qd_full(k,1);
    dt2 = qd_full(k,2);
    ddt1 = qdd_full(k,1);
    ddt2 = qdd_full(k,2);
    
    tau1 = 0.3333*ddt1*l1^2*m1 + ddt1*l1^2*m2 + 0.3333*ddt1*l2^2*m2 + 0.3333*ddt2*l2^2*m2 + 0.5000*g*l2*m2*cos(t1 + t2) + 0.5000*g*l1*m1*cos(t1) + g*l1*m2*cos(t1) - 0.5000*dt2^2*l1*l2;
    tau2 = 0.1667*l2*m2*(3*l1*sin(t2)*dt1^2 + 2*ddt1*l2 + 2*ddt2*l2 + 3*g*cos(t1 + t2) + 3*ddt1*l1*cos(t2));
    tau(k,:) = [tau1, tau2];
end

figure(1)
plot(tau);xlabel('time(s)');ylabel('joint torques');title('joint torque vs time');
legend({'tau1','tau2'})

%% joint torques as a function of time  

for a = 1:1:N
    t1 = q_full(a,1);
    t2 = q_full(a,2);
    dt1 = qd_full(a,1);
    dt2 = qd_full(a,2);
    ddt1 = qdd_full(a,1);
    ddt2 = qdd_full(a,2);
    
    % a. Joint torque resulted from the inertial element
    I1 = ddt1*(0.4167*l1^2*m1 + l1^2*m2 + 0.2500*l2^2*m2 + l1*l2*m2*cos(t2)) + ddt2*(0.0833*m1*l1^2 + 0.5000*m2*cos(t2)*l1*l2 + 0.2500*m2*l2^2);
    I2 = ddt1*(0.0833*m1*l1^2 + 0.5000*m2*cos(t2)*l1*l2 + 0.2500*m2*l2^2) + ddt2*(0.0833*m1*l1^2 + 0.2500*m2*l2^2);
    I(a,:) = [I1, I2];
    
    % b. Joint torque resulted from the centrifugal element
    cf1 = -0.5000*l1*l2*m2*sin(t2)*dt2^2;
    cf2 =  0.5000*dt1^2*l1*l2*m2*sin(t2);
    cf(a,:) = [cf1,cf2]; 

    % c. Joint torque resulted from the curiolis element
    cur1 = - dt1*l1*l2*m2*sin(t2)*dt2;
    cur2 = 0;
    cur(a,:) = [cur1,cur2];

    % d. Joint torque resulted from the gravitational element
    G1 = g*m2*(0.5000*l2*(cos(t1)*cos(t2) - sin(t1)*sin(t2)) + l1*cos(t1)) + 0.5000*g*l1*m1*cos(t1);
    G2 = 0.5000*g*l2*m2*(cos(t1)*cos(t2) - sin(t1)*sin(t2));
    G(a,:) = [G1,G2];
end


figure(5)
plot(I);xlabel('time(s)');ylabel('Joint torque resulted from the inertial element');title('Joint torque resulted from the inertial element vs time');
legend({'I1','I2'})
figure(6)
plot(cf);xlabel('time(s)');ylabel('Joint torque resulted from the centrifugal element');title('Joint torque resulted from the centrifugal element vs time');
legend({'cf1','cf2'})
figure(7)
plot(cur);xlabel('time(s)');ylabel('Joint torque resulted from the curiolis element');title('Joint torque resulted from the curiolis element vs time');
legend({'cur1','cur2'})
figure(8)
plot(G);xlabel('time(s)');ylabel('Joint torque resulted from the gravitational element');title('Joint torque resulted from the gravitational element vs time');
legend({'G','G'})

% e. Total joint toques 
Q = I+cf+cur+G;
figure(9)
plot(Q);xlabel('time(s)');ylabel('Total joint toques');title('Total joint toques vs time');
legend({'Q1','Q2'})

function [theta1, theta2] = IK(targetX, targetY, linkLength1, linkLength2)

    a = (targetX^2 + targetY^2 - linkLength2^2 - linkLength1^2) / (2 * linkLength1 * linkLength2);
    
    if abs(a) > 1
        error('Target is out of reach for the robot arm');
    end

    theta2 = atan2(sqrt(1 - a^2), a);

    k1 = linkLength1 + linkLength2 * cos(theta2);
    k2 = linkLength2 * sin(theta2);
    
    theta1 = atan2(targetY, targetX) - atan2(k2, k1);
end