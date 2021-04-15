% Initial Estimation of Kinematic Structure of a Robotic Manipulator
% Source code of the algorithms
% Repository: https://github.com/robot-vsb-cz/initial-estimation
% Licence: GPL-3.0

% Dependencies: 
% Robotics Toolbox by Peter Corke
% https://petercorke.com/toolboxes/robotics-toolbox/

close all;
clear all;
clc;

%% Inputs
% choose estimation type 1 - A, 2 - B, 3 - C, 4 - D
estType = 4;

% use prepared set of poses
load('poses.mat')
% or define an own pose
T(1) = [ -0.5005   -0.1800   -0.8468        -1
         -0.0654    0.9832   -0.1704       0.9
          0.8633   -0.0299   -0.5039       0.8
               0         0         0         1];

% create an SE3 object out of chosen pose
pose = SE3(T(1));
% create a base object (identity matrix if not specified)
base = SE3();

% choose number of joints (3 to 7)
nJoints = 4;

%% Perform estimation and plot results
switch estType
    case 1
        dh = estTypeA(base,pose,nJoints);
        mask = 1;
    case 2
        dh = estTypeB(base,pose,nJoints);
        mask = 1;
    case 3
        dh = estTypeC(base,pose,nJoints);
        mask = 0;
    case 4
        dh = estTypeD(base,pose,nJoints);
        mask = 0;
end

% create robot object a print table with DH in console
robot = makeCorkeRobot(dh,nJoints)

% inverse kinematics, apply mask for types A,B to ignore orientation
if mask == 0
    q = robot.ikunc(pose);
else
    q = robot.ikine(pose, 'mask', [1 1 1 0 0 0]);
end

% plotting
robot.plot(q);
hold on;
trplot(pose, 'color', 'red', 'length', 1.5);
hold off;