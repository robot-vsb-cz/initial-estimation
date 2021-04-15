function x0 = estTypeC(base,pose,nJoints)
% Type C initial estimation of kinematic structure for given pose

% define line 1 represented by base vector
L1 = zeros(2,3);
L1(1,:) = transpose(base.t);
L1(2,:) = transpose(base.t+0.2*base.a);

% define line 2 represented by pose vector
L2 = zeros(2,3);
L2(1,:) = transpose(pose.t);
L2(2,:) = transpose(pose.t+0.2*pose.a);

% calculate the nearest distance using common perpendicular and determine intersection points
[length, P, Q] = dist2lines(L1,L2);
P = transpose(P);
Q = transpose(Q);

% angle alpha between base Z vector and pose Z vector
angle = atan2(norm(cross(base.a, pose.a)), dot(base.a, pose.a));

% using right-hand rule, if a dot product of pose Zi+1 axis is in negative
% direction of base Yi axis, the angle has to be negative
if dot(base.o, pose.a) < 0
    angle = -angle;
end

% obtaining DH params
a = length/(nJoints-1);
an = 0;

al = angle/(nJoints-1);
aln = 0;

d0 = dot(P - base.t, base.a);
d = 0;
dn = dot(pose.t - Q, pose.a);

% prepare a variable
x0 = zeros(1,3*nJoints + 3);

switch nJoints
    case 3        
        x0(1,1:12) = [a a an d0 d dn al al aln base.t(1) base.t(2) base.t(3)];
    case 4
        x0(1,1:15) = [a a a an d0 d d dn al al al aln base.t(1) base.t(2) base.t(3)];
    case 5
        x0(1,1:18) = [a a a a an d0 d d d dn al al al al aln base.t(1) base.t(2) base.t(3)];
    case 6
        x0(1,1:21) = [a a a a a an d0 d d d d dn al al al al al aln base.t(1) base.t(2) base.t(3)];
    case 7
        x0(1,1:24) = [a a a a a a an d0 d d d d d dn al al al al al al aln base.t(1) base.t(2) base.t(3)];
end


end
