function [X, Y]= ForwardKinematics(Q2, Q4)
%    l1 = sqrt(0.316^2+0.088^2);
%    l2 = sqrt(0.384^2+0.088^2);
    l1 = 1;
    l2 = 1;

    X = -l1 .* sin(Q2) - l2.*sin(Q2-Q4);
    Y = l1 .* cos(Q2) + l2.*cos(Q2-Q4);
end