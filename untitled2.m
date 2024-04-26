% Import Robotics Toolbox
startup_rvc;

% Define the DH parameters
% Link = [theta, d, a, alpha, offset, qlim]
L(1) = Link('d', 400, 'a', 0, 'alpha', 0, 'qlim', [-170/180*pi 170/180*pi]);
L(2) = Link('d', 0, 'a', 325, 'alpha', 0, 'qlim', [-145/180*pi 145/180*pi]);
L(3) = Link('d', 0, 'a', 225, 'alpha', 0, 'qlim', [-2*pi 2*pi]);
L(4) = Link('theta', 0, 'a', 0, 'alpha', 0, 'offset', -150, 'qlim', [0 150], 'prismatic');
L(5) = Link('d', -170, 'a', 0, 'alpha', 0);

% Create the robot model
robot = SerialLink(L, 'name', 'MyRobot');

% Display the robot
robot.teach();

% 目标位置的齐次变换矩阵
T1 = [1 0 0 490; 
      0 1 0 -45; 
      0 0 1 100; 
      0 0 0 1];

% 尝试解决逆运动学问题，包括掩码矩阵
q = robot.ikine(T1, 'mask', mask);

% 检查解中是否有 NaN 值来确定是否找到有效解决方案
if any(isnan(q))
    disp('无法到达该点');
else
    disp('可以到达该点');
    disp(q); % 显示关节参数
end
