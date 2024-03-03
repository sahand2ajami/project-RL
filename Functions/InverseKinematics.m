function [Q2, Q4] = InverseKinematics(X, Y)

%     l1 = sqrt(0.316^2 + 0.088^2);
%     l2 = sqrt(0.384^2 + 0.088^2);
    l1 = 1;
    l2 = 1;
    
    Q4 = -(pi-acos((-X.^2 - Y.^2 + l1.^2 + l2.^2)./(2.*l1.*l2)));
    Q2 =  atan2((l2.*sin(Q4)), (l2.*cos(Q4)+l1)) - atan2(X, Y);

end