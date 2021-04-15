function x0 = estTypeD(base,pose,nJoints)
% Type D initial estimation of kinematic structure for given pose


% control points parameter
paramCP = norm(pose.t-base.t)/2;

% define Bezier points
bezierPoints = zeros(3,4);
bezierPoints(:,1) = base.t;
bezierPoints(:,2) = base.t + paramCP*base.a;
bezierPoints(:,3) = pose.t - paramCP*pose.a;
bezierPoints(:,4) = pose.t;

% approximate Bezier spline, number of points = number of joints
t = linspace(0,1,nJoints);
Q3D = Bezier(bezierPoints,t);

% prepare SE3 joint variables
joints(nJoints+1) = SE3();
joints(1) = base;
for i = 2:nJoints
    joints(i).t = Q3D(:,i);
end
joints(nJoints+1) = pose;

% defining transformation for joints except the last one, starting with the second
for i = 2:nJoints-1
    % start in the same transformation as the previous joint
    joints(i) = joints(i-1);
    % translate frame on the Bézier curve changing its translation vector t
    joints(i).t = Q3D(:,i);
    
    % expres the frame in the coordinate frame of the previous joint using its inverse matrix
    jointBase = inv(joints(i-1))*joints(i).t;
    jointBase = jointBase - [0; 0; jointBase(3)];
    
    theta(i) = atan2(norm(cross([1; 0; 0], jointBase)), dot([1; 0; 0], jointBase)); 
    
    % using right-hand rule, if a dot product of Xi axis is in negative
    % direction of Yi-1 axis, the angle has to be negative
    if dot([0; 1; 0], jointBase) < 0
        theta(i) = -theta(i);
    end
    
    % update the tranformation matrix by rotation around theta
    joints(i) = joints(i)*joints(i).Rz(theta(i));

    % expres the position of the next joint base vector t in the current coordinate frame 
    nextJointBase = inv(joints(i))*joints(i+1).t;
    nextJointBase = nextJointBase - [nextJointBase(1); 0; 0];
    
    alpha(i) = atan2(norm(cross([0; 0; 1], nextJointBase)), dot([0; 0; 1], nextJointBase));
        
    % using right-hand rule, if a dot product of Zi-1 axis is in negative
    % direction of Yi axis, the angle has to be negative
    if dot(cross(nextJointBase, [1; 0; 0]), [0; 0; 1]) < 0
        alpha(i) = -alpha(i);
    end
    
    % update the tranformation matrix by rotation around alpha    
    joints(i) = joints(i)*joints(i).Rx(alpha(i));  
end

% define line 1 represented by n-1 joint Z vector
L1 = zeros(2,3);
L1(1,:) = transpose(joints(nJoints-1).t);
L1(2,:) = transpose(joints(nJoints-1).t+0.02*joints(nJoints-1).a);

% define line 2 represented by pose vector
L2 = zeros(2,3);
L2(1,:) = transpose(pose.t);
L2(2,:) = transpose(pose.t+0.02*pose.a);

% calculate the nearest distance using common perpendicular and determine intersection points
[length, P, Q] = dist2lines(L1,L2);
P = transpose(P);
Q = transpose(Q);
PQ = Q-P;

% angle theta between joints i-1 X vector and pose X vector
% rotZ = atan2(norm(cross(joints(nJoints-1).a, pose.a)), dot(joints(nJoints-1).a, pose.a));
rotZ = atan2(norm(cross(joints(nJoints-1).n, PQ)), dot(joints(nJoints-1).n, PQ));

% using right-hand rule, if a dot product of Xi axis is in negative
% direction of Yi-1 axis, the angle has to be negative
if dot(joints(nJoints-1).o, PQ) < 0
    rotZ = -rotZ;
end

% update the last joint transformation with calculated rotation theta and translation (dn, an params)
joints(nJoints) = joints(nJoints-1);
joints(nJoints) = joints(nJoints)*joints(nJoints).Rz(rotZ);
joints(nJoints).t = Q;

% angle alpha between joints i-1 Z vector and pose Z vector
rotX = atan2(norm(cross(joints(nJoints).a, pose.a)), dot(joints(nJoints).a, pose.a));

% using right-hand rule, if a dot product of Zi+1 axis is in positive
% direction of Yi axis, the angle has to be negative
if dot(joints(nJoints).o, pose.a) > 0
    rotX = -rotX;
end

% update the transformation matrix with rotation alpha
joints(nJoints) = joints(nJoints)*joints(nJoints).Rx(rotX);

% obtain DH between joints
for i = 1:nJoints
    [d, th, a, al] = getDH(joints(i),joints(i+1));
    dh(:,i) = [d; th; a; al];
end

% prepare a variable
x0 = zeros(1,3*nJoints + 3);

switch nJoints
    case 3        
        x0(1,1:12) = [dh(3,:) dh(1,:) dh(4,:) base.t(1) base.t(2) base.t(3)];
    case 4
        x0(1,1:15) = [dh(3,:) dh(1,:) dh(4,:) base.t(1) base.t(2) base.t(3)];
    case 5
        x0(1,1:18) = [dh(3,:) dh(1,:) dh(4,:) base.t(1) base.t(2) base.t(3)];
    case 6
        x0(1,1:21) = [dh(3,:) dh(1,:) dh(4,:) base.t(1) base.t(2) base.t(3)];
    case 7
        x0(1,1:24) = [dh(3,:) dh(1,:) dh(4,:) base.t(1) base.t(2) base.t(3)];
end

end
