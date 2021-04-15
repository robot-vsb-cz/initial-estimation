function x0 = estTypeA(base,pose,nJoints)
% Type A initial estimation of kinematic structure for given pose


% norm of the pose and base points in XY base plane
length = sqrt((pose.t(1)-base.t(1))^2 + (pose.t(2)-base.t(2))^2);

% angle for alpha guess
angle = pi/4;

% obtaining DH params
a = length/nJoints;
d1 = pose.t(3)-base.t(3);
d = 0;
dn = 0;

% prepare a variable for the initial estimation values
x0 = zeros(1, 3*nJoints + 3);

switch nJoints
    case 3
        al = angle/2;
        x0(1,1:12) = [a a a d1 d dn -al 2*al -al base.t(1) base.t(2) base.t(3) ];
    case 4
        al = angle;
        x0(1,1:15) = [a a a a d1 d d dn al -al al -al base.t(1) base.t(2) base.t(3) ];
    case 5        
        al = angle/2;
        x0(1,1:18) = [a a a a a d1 d d d dn al -2*al al -al al base.t(1) base.t(2) base.t(3)];
    case 6
        al = angle;
        x0(1,1:21) = [a a a a a a d1 d d d d dn al -al al -al al -al base.t(1) base.t(2) base.t(3)];
    case 7
        al = angle/2;
        x0(1,1:24) = [a a a a a a a d1 d d d d d dn al -2*al al -al al -al al base.t(1) base.t(2) base.t(3)];
end

end

